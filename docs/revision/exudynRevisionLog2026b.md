# Exudyn Revision Log 2026b

What was done and found in [`exudynRevisionPlan2026b.md`](exudynRevisionPlan2026b.md), in
plan order: one entry per closed step, with the anchor of its number (`<a id="rg8-1"></a>`),
a heading `### RG8.1 — title (date)`, and what the work actually turned out to be — the
measurements, the surprises, and what was decided against.

The log of the first plan, [`exudynRevisionLog2026.md`](exudynRevisionLog2026.md), is closed:
it records a revision that completed as 1.12 and is never appended to again. **A closed entry
is never edited** in either log; a fact that turns out wrong is corrected in
[`exudynRevisionInfo2026.md`](exudynRevisionInfo2026.md), which both plans share, with a dated
note here.

<a id="rg3-1"></a>
### RG3.1 — the manual gets its structure back (2026-09-22, #2584)

The conversion of revision2026 step R7.1.5 put the `toctree` of `introduction.md` **below its
last section**. Sphinx nests what a toctree lists under the section it stands in, so the
documentation showed *Exudyn Basics*, *Advanced topics* and *C++ Code* as sub-pages of
**"Mapping between local and global coordinate indices"** — three chapters filed under a
paragraph about index arithmetic. It built without a warning, because nothing about it is
invalid; it was simply wrong, and it stayed wrong for a week until the maintainer read the
sidebar.

**What the manual looks like now** (maintainer, 2026-09-22):

```
README
Installation and Getting Started     <- was after "Overview on Exudyn"
Overview on Exudyn
  Items: Nodes, Objects, Loads, Markers, Sensors, ...
    ... Reference coordinates and displacements
    Mapping between local and global coordinate indices   <- was a chapter-level section
Exudyn Basics                        <- was a sub-page
Advanced topics                      <- was a sub-page
Tutorial, Graphics, the command line, ...
```

**The C++ chapter is gone, and that is the part worth recording.** It held three kinds of
material, and two of them existed twice:

- *the four principles* — developer-friendly, error minimization, user-friendliness,
  efficiency, in that order, and what follows from the order — existed **only** there. It is
  the "why" of the C++ side and is now the opening section of `docs/dev/ARCHITECTURE.md`.
- *code style, notation conventions, the no-abbreviations rule* — `docs/dev/CODING_STYLE.md`
  §1-§5 carries all of it, in tables, and is the document that is maintained. Dropped.
- *the module list and the entry points* — `ARCHITECTURE.md` has a current table; the
  manual's list still named `Objects:` and `pythonGenerator`. Dropped.
- *how to implement a new item in C++* — the opposite case: the manual had **two worked
  cases** (a body and a connector, with every function to implement and what each one is for),
  while `CODING_STYLE.md` §9 said *"TBD after the current 2026 revision"*. That section now
  carries them, with the file names of the current layout (`definitions/itemDefs<Kind>.py`,
  `python tools/regenerate.py`, `msvc/cppsrc.vcxproj`) and the advice that came with them:
  **write it in Python first with user functions, and go to C++ when that works and is too
  slow.**

What the manual keeps is a short **"The C++ core"** section in *Advanced topics*: what happens
in C++ at all, and two links into the developer documentation for the two questions people
actually ask. A user of a wheel needs no more; a developer was reading the wrong copy.

The structure was invisible to every check — `-W` catches a broken reference, not a chapter
in the wrong place. This one needed a person to look at the sidebar.

<a id="rg3-2"></a>
### RG3.2 — five how-to notes go back to being repository files (2026-09-22, #2585)

`docs/howTo/` holds eight notes, and revision2026 step R7.1.2 published all eight because all
eight are Markdown. Being readable is not the same as being documentation: five of them answer
questions that only somebody who *maintains* Exudyn ever asks.

| stays a page | why |
|---|---|
| `buildFromSource.md` | a user builds from source; `README.rst` and `CONTRIBUTING.md` both send people here |
| `condaEnvironments.md` | the environments, the package→feature map, the scipy pin |
| `sphinxDocs.md` | a first contributor who changes a page has to build it |

| leaves the build | why |
|---|---|
| `buildQuirks.md` | `/bigobj`, ABI tags, a startup crash under the debugger |
| `gccVsMsvcTraps.md` | what MSVC accepts and GCC does not |
| `visualStudio2022.md` | which VS2022 components to tick |
| `convertVideosFfmpeg.md` | producing the demo animations |
| `matplotlibExamples.md` | the plotting recipes of the examples |

The mechanism is `exclude_patterns` in `conf.py` — the list that already keeps `docs/revision/*`,
`CLAUDE.md` and `.github/*` out. An excluded file is still in git, still readable on GitHub and
still linked; it is simply not a page.

**The work was not the exclusion, it was the links.** A relative Markdown link into an excluded
file is a link to a document that does not exist, and the strict build fails on it — which is why
revision2026 step R7.1.2, when these same files were excluded for a different reason, linked them
to GitHub from the developer index. Nine links needed that treatment: four in `buildFromSource.md`
(it is the page that sends a reader to the Windows quirks, the GCC traps and Visual Studio) and
five in the how-to table of `docs/dev/README.md`, which is now **two** tables: three pages and
five repository notes, so that the table itself says which is which.

`index.md` keeps the how-to section with the three pages and one paragraph naming the other five
with a link to the directory — the "mentioned in one line" the maintainer asked for.

What this does **not** change: nothing is deleted, nothing moves, and a maintainer who clones the
repository has exactly what they had. What a reader of the published documentation no longer finds
is five pages of instructions for a machine they are not sitting at.

<a id="rg3-4"></a>
### RG3.4 — the revisions chapter already said it (2026-09-22, #2587)

Nothing to do. The step was raised from the maintainer's list of 2026-09-22 and asks the
revisions chapter to point at the developer documentation and to say that there are two plans
now. [`docs/manual/revisions.md`](../manual/revisions.md) has carried exactly that since commit
`f7896d1` of the same day, written while revision2026b was being set up and a few hours before
the list was read:

> This chapter is deliberately short. The revision behind it is recorded **step by step** in a
> plan and a log, which are working documents of the project rather than user documentation:
> there are two of them, *revision2026* - finished, and what version 1.12 is - and
> *revision2026b*, which carries what it did not finish. Both are linked from the
> [developer documentation](../dev/README.md), together with the standing information document
> that holds the measured facts and the decisions.

#2587 was therefore already RESOLVED when this step was reached, stamped 1.12.2. What this entry
records is the **check**, not a change: the wish and the text agree, including the detail that
the published page does not link `docs/revision/*` directly, because those files are excluded
from the build.

<a id="rg3-5"></a>
### RG3.5 — the citations become links (2026-09-22, #2550)

**54 citations across 25 pages** pointed nowhere. `[Hairer1987]`, `[ZwoelferGerstmayr2021]`,
`[Chung1993]` were printed as literal text in the middle of sentences that name them as
references - in the hand-written chapters, and in the item pages, where they come from the
`\cite{...}` commands still standing in `definitions/`. The LaTeX build resolved them; nothing
has since.

**What was missing was only the destination.** `docs/bibliographyDoc.bib` survived - 100 entries,
moved to `docs/` in revision2026 step R7.1.7 - and covers **every** key the documentation uses:
the emitter checks that, and reports a key it cannot find.

**The mechanism, which is the part worth reading.** A new emitter,
`tools/generators/referencesDocsEmitter.py`, writes `docs/generated/references.md`, where every
entry carries a target `(ref-<key>)=`. Then `conf.py` appends **one Markdown link definition per
key** to every Markdown document Sphinx reads:

```
[Hairer1987]: #ref-hairer1987
```

CommonMark resolves `[Hairer1987]` as a *shortcut reference link*, so the citation becomes a link
with **no source text edited at all** - not in the 24 manual chapters, not in the 450 generated
pages, not in `definitions/`. Two properties follow from letting the Markdown parser do it:

- a key inside a code fence or inline code stays literal text, because the parser knows it is
  code. A regular expression over the sources would not have known;
- a definition that no page uses produces nothing, so all 100 can be appended everywhere.

The files on disk are untouched: the hook runs on the text Sphinx has already read.

**What it cost to get the bibliography readable.** The `.bib` is LaTeX: `N{\o}rsett`,
`Br{\"{u}}ls`, `{M}ehrk{\"o}rpersysteme`, `\url{...}`, `--` for a page range, and titles wrapped
in an extra brace pair. `CleanLatex()` undoes it, with one rule that matters: `$...$` is **masked
out first and put back afterwards**, so the braces of a title are stripped while the braces of
`$x_{gap}$` survive - "the generalized-$\alpha$ scheme" stays a formula instead of becoming a
backslash. Parsed by hand rather than with `bibtexparser`: thirty lines against a new dependency
(rule 6).

Two characters were repaired **in the `.bib` itself**, not worked around in the emitter: an en
dash in "St. Venant-Kirchhoff" and one in a page range had become U+FFFD, the replacement
character, in an encoding conversion long ago.

**What remains.** The page lists the whole bibliography, 100 entries, of which 54 are cited: the
emitter's only input is the `.bib`, so its output never depends on the state of another generated
page. And nothing *enforces* that citations resolve - if the hook in `conf.py` were removed, they
would quietly go back to being text. The guards today are the drift check (the page and the key
list are generated files) and this entry.

<a id="rg3-6"></a>
### RG3.6 — a table with a header and no rows (2026-09-22, #2592)

Found while trying to build the PDF, and a defect in its own right. `structureDocsEmitter.py`
writes the five column headings of a settings table **before** the loop that writes the rows, and
the loop skips a parameter that is deprecated or has no pybind interface. `VSettingsWindowDeprecated`
has nothing else left, so the published page showed five headings with an empty box under them.

The LaTeX writer of Sphinx does `next(node.findall(nodes.tbody))` on a table and raises
`StopIteration` when there is no body — sphinx/builders/latex/transforms.py:443. That is not a
warning, it is a traceback, and it aborted the whole build. One table in 450 generated pages.

The emitter now remembers where the header started and takes it back if no row followed, writing
*"(none: this structure has no items in the Python interface...)"* instead. Generic: it is the
zero-row case that is handled, not that one structure.

<a id="rg3-7"></a>
### RG3.7 — the sentence was the formula and the formula was the source (2026-09-22, #2593)

The worst of what the PDF uncovered, because it had been wrong on the **published HTML** for as
long as the Markdown has existed, and nobody had looked:

```
- $g = L - r_i + r_j$: $$
    \termC{\diffmOI{g} = \LU{0}{\nv_0}\!\tp \LU{0}{\Jm_{pos}} }

$$
```

A display math block opened at the end of a line of prose. MyST reads that as: the **sentence**
is the formula (it was typeset as one, `\[ - $g = L - r_i + r_j$: \]`), and the **formula** is a
code block of raw LaTeX. 45 such blocks in three documents. A reader of the contact theory saw
`\termC{\diffmOI{g} = ...}` printed as text.

Two causes, and each needed its own fix:

- **the converter** (`tools/generators/latexToMarkdown.py`): `ConvertDisplayMath` runs before
  `ConvertLists`, and `ProtectMath` has replaced each block with a one-character placeholder by
  then, so the list conversion — which joins an item's lines into one — joined the formula
  into the sentence. Display math now gets a placeholder of its own kind, which `ConvertLists`
  keeps on its own line. Two smaller things with it: the body was stripped of newlines but not of
  the indentation, which left a line of blanks before the closing `$$— ` and a blank line ENDS a
  block in MyST; and `\nonumber`, left over from the eqnarray days, is ignored by MathJax and is a
  hard LaTeX error inside `aligned`.
- **the chapter** (`docs/manual/theoryContact.md`): written that way by the conversion of
  revision2026 step R7.1.5 and hand-written since, so it had to be repaired in place. All 87
  display blocks of the file were normalised to the same shape, keeping the two `$$ (eq-...)`
  labels on their closing line — losing those is what the strict html build caught when a first
  attempt dropped them.

42 `\nonumber` came out of four chapters, 30 more out of `definitions/`. The item pages now read
as intended, with the list numbering continuing across the formulas.

<a id="rg3-3"></a>
### RG3.3 — the PDF, from the Markdown (2026-09-22, #2586)

**1159 pages, 10.1 MB, zero LaTeX errors.** `exudev docs --pdf`. The old `theDoc.pdf` was 1090
pages and 8.6 MB.

Decision D8 ended the PDF with the LaTeX sources, and D8 was right about the sources: nothing is
written in `.tex` any more, and nothing here changes that. What comes back is a **rendering**, and
what decided it was a measurement rather than an opinion — the corpus was put through
`sphinx -b latex` before anything was planned. It crashed once (RG3.6) and otherwise produced a
complete `.tex` with 22 warnings in two categories. The premise of D8 — that a PDF means a
second rendering of every page to maintain — turned out not to hold for Markdown sources.

**What is in it** (maintainer, 2026-09-22): everything except the 517 pages of example and test
model source listings, and **including the issue history**, which is the part that makes the
document worth searching — and worth feeding to an AI tool, which was the maintainer's actual
argument. The last chapter is the tracker log.

**How the two builds stay one build.** `sphinx -b latex . _buildpdf/latex -t pdf`: the tag is the
only difference. `conf.py` branches on it once — `master_doc` becomes `pdfIndex.md`, and
`index.md`, `README.rst` and the two listing directories are excluded. The html build does not
see `pdfIndex.md` at all. Nothing else is conditional.

**The front page.** `README.rst` cannot be in a PDF: ten of its images are badges fetched from the
web **as SVG**, and three are animated GIFs. `pdfIndex.md` replaces it — what Exudyn is, how to
cite it, where the living documentation is, and what this document leaves out. It carries the
toctrees, and therefore it is the one place the order of the PDF is written down.

**The math, which was the real question.** The `.tex` uses `\qv` 544 times and `\LU` 2628 times,
and LaTeX knows neither: they are declared to MathJax in `conf.py`, in the dict that revision2026
step R7.1.5 made the single place for them. The preamble is now **generated from that same dict**
— 162 `\newcommand`s written by `conf.py` itself. There is no second list to keep in step, and
`tools/checkMathMacros.py` still guards the first one.

Three things had to be decided by hand, and are written out rather than guessed:

- **five names are LaTeX commands already** — found by asking LaTeX (`\ifdefined` over all 162),
  not by reading. `vspace` keeps its LaTeX meaning (MathJax has a no-op for it, and Sphinx's own
  output uses the real one); `AE`, `Im`, `mp` and `vec` are redefined, because here the project's
  meaning must win: a Lagrange multiplier and not a ligature, an identity matrix and not the
  imaginary part, a 2x2 matrix and not the minus-plus sign, the vec() operator and not an arrow
  accent. Everything else is `\newcommand`, which **fails loudly** if a macro added later collides.
- **`\ddot \mathbf{q}`** is what `acc` was defined as. MathJax reads it as intended; LaTeX reads
  `\ddot{\mathbf}` and stops. The body is braced properly now — in the dict, so both consumers
  get correct input.
- **`<br>` in a table cell.** The settings tables stack the access paths of an item with a raw
  `<br>`, 468 of them. Raw html is html: the LaTeX writer drops it, and the paths would run
  together. A post-transform in `conf.py` translates it to `\newline`, for the latex builder only,
  so the emitters keep writing the html that is right for the html.

**xelatex, not pdflatex.** The pages hold 26 distinct non-ASCII characters, among them the
box-drawing `─│└├` of the directory trees (109 of them) and `→` (130). pdflatex has no glyph
for those; xelatex rendered the document with **zero** missing characters.

**mermaid.** The twelve flowcharts are rendered to PDF by `mermaidx`, a pure-Python renderer —
no Node, no headless browser — in a new dev-only `pdf` dependency group. The html build renders
the same diagrams in the browser and needs none of it.

**Where it is not.** Not in any gate. `exudev docs` alone is unchanged, `exudev build --complete`
does not build it, and Read the Docs cannot: it needs a LaTeX installation. It is built for a
release (`Complete()` passes `pdf=True` only when the release checks are on), lands in `dist/`
beside the wheels as `exudynDocumentationV<version>.pdf`, and the release checklist names it as
an asset to attach.

**One deviation from the plan**, for the record: `sphinx -M latexpdf` was the intended single
step, and it does not work on Windows — it shells out to a `make.bat` that wants a `make` MiKTeX
does not have. The .tex build and the LaTeX run are two steps now, which is better anyway: the
summary shows which of the two failed, and the one that needs an installation outside Python is
visible as such.

<a id="rg6-2-1"></a>
### RG6.2.1 — the dialogs leave the C++ (2026-09-22, #2595)

`src/Main/rendererPythonInterface.cpp` went from **775 lines to 528**, and the 220 of them that
were Python are Python now. The file had six `R"PY(...)PY"` literals: the help dialog (69 lines),
the command window (10 + 93), the quit question (29), the right-mouse dialog (built by C++ string
concatenation) and the one-line call that the settings dialog already was. Five calls of five
lines each are left.

What a raw string literal costs is not hypothetical. One of the blocks carried

```
    tkWindow.attributes('-topmost', True) #puts window topmost(permanent)\n";
```

— a fragment of the C++ that wrote it, sitting inside the Python, harmless only because it
landed after a `#`. No syntax check, no ruff, no stub check and no import test had ever looked at
any of it.

**Nothing changes for a user.** The same settings decide the same things: the window setup that
the C++ assembled by concatenating `visualizationSettings.dialogs.alwaysTopmost` and
`alphaTransparency` into the source text is now `ApplyDialogWindowSettings()`, which reads them
through `GetRendererSystemContainer()` — which is how `EditDictionaryWithTypeInfo` has always
done it. The quit question still answers through `exudyn.sys['quitResponse']` as 2 or 3, and each
dialog prints what it printed before when tkinter is missing.

**What is now possible and was not**: the utility documentation emitter picked the five functions
up by itself, so they have a page; ruff reads them; `allExudynModulesTest.py` imports them with
the rest of the package; and the two paths that do NOT need a window —
`ShowVisualizationSettingsDialog` with no renderer attached, `ShowRightMouseSelectionDialog` with
no selection — were run here and print what the renderer expects.

**Not tested, and it cannot be**: everything that opens a window. That is what RG6.2.2 is for —
the layer under the widgets, which needs no window at all.

**The help text moved as it was**, 48 lines of key bindings. It is the third copy of the same
knowledge — `GlfwClient.cpp` implements the keys, `docs/manual/GUI.md` tabulates them in 64
rows — and moving it does not fix that. It is now a module constant rather than a string inside
C++, which is the position a generator would need; RG6.2.6 records the rest.

<a id="rg6-2-2"></a>
### RG6.2.2 — the value layer gets tests, and the tests find three defects (2026-09-23, #2596)

`python/testing/test_guiValues.py`, 41 tests. They cover the four functions that decide what a
typed value becomes — `ConvertString2Value`, `ConvertValue2String`, `CheckType`,
`GetComboBoxListsDict` — which had no test at all, and which are the only part of the dialog
that can be tested, because everything above them opens a window.

**The test that matters is not invented data.** It walks the real settings structures — 622
values between `simulationSettings` and `visualizationSettings` — and puts each one through the
round trip the dialog performs on it: `ConvertValue2String` on the way in, `CheckType` and
`ConvertString2Value` on the way out. A value that does not survive is one a user cannot open the
dialog on without changing it.

**Five of the 622 do not survive**, and each is a defect (#2597, to be fixed in RG6.2.3):

- **every enum value is rejected.** `CheckType` has no branch for an enum type, so
  `LinearSolverType.EXUdense` falls through to its `exec()` fallback, raises `NameError` and
  comes back as *"invalid array or matrix: check brackets and types"*. It does not show today
  only because an enum is edited through the **combo box**, which calls `ConvertString2Value`
  directly and never asks `CheckType`. The entry field and the combo box disagree about what is
  valid, and nothing said so.
- **a file name cannot be an absolute Windows path.** `:` is not in `validFileNameChar`, so
  `C:/anything` is refused — including
  `interactive.openVR.actionManifestFileName`, whose **shipped default** is
  `C:/openVRactionsManifest.json`.
- and, from reading rather than from the round trip: a value that passes `CheckType` but fails
  `ConvertString2Value` — `-3` for a `UInt`, which `CheckType` does not range check — is dropped
  by `GetDictionary` with a `print()` to the console. The dialog accepts the edit and the setting
  never changes.

**The five are a list in the test file, and it is meant to shrink.** A path that starts working
fails the test until it is taken out of `knownRoundTripGaps`; a path that stops working is a new
failure. That is the same shape as the stubtest baseline, and it was checked by removing one
entry and watching the test go red.

**One more gap is recorded as an `xfail`**: `GetComboBoxListsDict` names three enum types by
hand, and `timeIntegration.explicitIntegration.dynamicSolverType` is not one of them, so it is
edited as free text. The test turns green by itself when RG6.2.3 builds the lists from the type
name.

What the tests also pin down, so that RG6.2.3 and RG6.2.4 can change the dialog without guessing:
`bool` is compared with the string `'True'` rather than parsed (so *anything else* is `False`),
the range checks live in the type name (`PReal` > 0, `UReal` >= 0, ...) and their messages name
it, floats are written through `float32` because the C++ side is single precision
(`1/3` -> `0.33333334`), and an enum is read back by comparing `str(value)` with the text.

<a id="rg3-9"></a>
### RG3.9 — three corrections to the landing pages and the developer chapters (2026-09-23, #2598)

**How Exudyn is developed is now on the first page.** Since version 1.11.0 it is developed
heavily with Anthropic's Claude Code — code, workflows, documentation, tests and examples —
and until today the documentation did not say so anywhere. The line stands directly under the
subtitle of `README.rst`, which is three pages at once (the GitHub landing page, the PyPI page
and the first page of the html documentation), and under the subtitle of `pdfIndex.md`, which is
the front page of the PDF.

**No hand-counted numbers in published text.** `pdfIndex.md` said the PDF leaves out *"the source
text of the 172 examples and 117 test models"* and that this is *"500 pages of Python"*. All three
numbers were right on the day they were written and are wrong as soon as somebody adds an example.
The rule the maintainer states: **a number that is not generated does not belong in published
text**. It now says "the examples and the test models" and "a large body of Python". The
measurements stay where they belong — in this log, in the plan and in the issue, each with the
date it was taken.

**The developer documents were chapters beside their own index.** In the PDF, *Exudyn developer
documentation* was a chapter and so were the seven documents it introduces —
`ARCHITECTURE`, `CODING_STYLE`, `WORKFLOW`, the three generator READMEs and `CONTRIBUTING— `
because `index.md` and `pdfIndex.md` listed all eight as siblings of one toctree. The fix is
where RG3.1 found the same lesson: a document that introduces others has to **carry** them.
`docs/dev/README.md` now holds a hidden toctree of the seven, placed **before its first section**
so that they attach to the document and not to a paragraph, and both tables of contents list only
`docs/dev/README`. The visible table of documents at the top of that page is unchanged — it is
what a human reads; the toctree is what Sphinx reads.

In the PDF they are `\section` under one `\chapter` now, with their own headings one level
deeper. The html sidebar nests the same way, which is the point: one structure, two renderings.

<a id="rg10-2"></a>
### RG10.2 — the issue table says what it shows (2026-09-23, #2600)

Three small things in `tools/issueTracker/issueServer.py`, and one of them was a column that
already existed everywhere except on the screen.

**A header, and it sticks.** The list was `<table><tbody>` and nothing else: five columns of
bare tags with no names anywhere on the page. It has a `<thead>` now — number, status, type,
effort, priority, title — and the header row is `position: sticky`, because the pane scrolls
and a column name that scrolls away is a column name that is not there.

**What the values mean is one hover away, and this page does not know them.** Each column name
carries a tooltip built from the tracker's own vocabularies, which `/api/meta` already sends:
*effort — LOW: within 2 hours, MEDIUM: within 16 hours, HIGH: within 40 hours, HUGE: above 40
hours*. If a vocabulary changes in `issueTracker.py`, the tooltip changes with it.

**The priority was never missing from the data.** `/api/issues` sends it in every row and the
filter bar filters on it; only the row renderer left it out. One `<td>`.

**`LOW EFF` and not `LOW`.** Effort and priority share their spelling — `LOW` and `HIGH` are
values of both — so two bare tags in one row cannot be told apart. The effort tag carries the
word, the priority tag stays plain, as the maintainer asked.

**Two tests, and the second one matters beyond this step.** The first checks that the columns
have names, that the effort tag says `EFF` and that a priority is drawn. The second checks that
the page script **parses**, through `quickjs— ` which arrives with `mermaidx` in the `pdf`
dependency group, and which the test skips when it is absent. Defining a function parses its body
without running it, so a missing `document` is not an error and a misplaced brace is. That is
exactly the check #2574 needed — a syntax error kills the whole script, error handlers
included, and the page shows a heading and nothing else — and the two headless browser tests
cannot give it at the moment, because the Edge of this machine updated under itself and now
prints no DOM for any address. Verified by breaking the script on purpose and watching the check
catch it.

<a id="rg6-2-3"></a>
### RG6.2.3 — the validator agrees with the settings (2026-09-23, #2597)

The first half of RG6.2.3: the three defects that RG6.2.2's round trip found, and the enum lists
that the last of them needed. `python/testing/test_guiValues.py` had a list of five settings that
did not survive being shown and read back; **the list is empty now**, and the test fails if
anybody puts an entry back that works.

**An enum is a value of a list, and `CheckType` did not know that.** It took no combo lists, so
`LinearSolverType.EXUdense` fell into its `exec()` fallback, raised `NameError` and came back as
*"invalid array or matrix: check brackets and types"*. It never showed, because an enum is edited
through the combo box and the combo box calls `ConvertString2Value` directly — so the entry
field and the combo box disagreed about what is valid, and nothing said so. `CheckType` takes the
lists now and rejects a wrong enum by **naming every value it may take**.

**`:` is a file name character.** It was not in `validFileNameChar`, so `C:/models/gear.stl` was
refused — and so was the dialog's own default for
`interactive.openVR.actionManifestFileName`, which is `C:/openVRactionsManifest.json`. A dialog
that rejects the value it is showing is the clearest kind of wrong.

**An edit that is out of range no longer disappears.** `CheckType` judges the SHAPE of a value;
the RANGE lives in the type name — `PReal` > 0, `UInt` >= 0 — and only `ConvertString2Value`
knows it. `-3` for a `UInt` therefore passed the check, went into the tree, and was dropped again
by `GetDictionary`, which printed *"illegal value"* to a console nobody is looking at while the
setting kept its old value. `OnEditEntryItem` now asks `ConvertString2Value` as well and puts its
message in the error box.

**And the enum lists come from the module.** `GetComboBoxListsDict` named three types by hand —
`OutputVariableType`, `LinearSolverType`, `ItemType— ` so
`timeIntegration.explicitIntegration.dynamicSolverType` was edited as free text, where a typo is
a silent wrong value. A pybind11 enum is recognised by its `__members__`, so the dict is built by
walking the module: **14 enum types instead of 3**, and an enum added to exudyn arrives in the
dialog by itself. That is what closed the fifth gap, and the `xfail` that recorded it became an
ordinary assertion.

One thing worth remembering from this step: the docstring of a changed function has to keep the
house shape — prose first, then `Args:`, then `Returns:`. A paragraph written after `Returns:`
stopped `utilityDocsEmitter.py` with a clear message, which is the generator doing its job.

**RG6.2.3, second half** (2026-09-23, #2601): the part a user sees.

**The columns have widths.** `tree.column(...)` was never called, so all of them kept tkinter's
200 px default: the name was cut, the description was unreadable, and dragging one moved the
others. Name, value and type keep what they are given (scaled with the display scaling), the
description stretches with the window.

**The type is a column.** It was read into `typeStorage` at load and never shown, although it is
what tells a reader whether to type `3`, `3.0`, `True` or `[1,2,3]`. The error box names it too
now — *"general.textSize expects PFloat: invalid float number"* instead of only *"invalid float
number"*.

**The description follows the mouse.** It was behind the key `h` and a modal message box, and the
column heading said so: *"Description (press H to show)"*. It is a tooltip beside the pointer now,
with the name and the type above it; the key still works. `Tooltip` is a small class in the same
module — tkinter has none — and it builds its window the first time it is needed, so a dialog
nobody hovers never creates one.

**What could not be verified here**: anything that opens a window. The module parses, the round
trip tests pass, every row write now carries three values and `GetDictionary` still reads the
value at index 0 — but whether the dialog *looks* right is for the maintainer to say, which is
why this half was handed over as soon as it built.

<a id="rg6-2-3-1"></a>
### RG6.2.3.1 — one font scaling for every platform (2026-09-23, #2602)

`dialogs.fontScaling` replaces `dialogs.fontScalingMacOS`, and the rule is the maintainer's:
**0 means what this platform did before the setting existed** — a fixed factor on MacOS, the
system display scaling on Windows and Linux — and any value above 0 sets the font factor and
the row height on **every** platform. So nothing changes for anybody who does not touch it, and a
Linux desktop finally has the knob: off MacOS the code said `if not IsApple(): fontFactor = 1`
and nothing could change that.

`fontScalingMacOS` is deprecated rather than removed, through the mechanism
`visualizationSettings` already has — `cFlags=SFDeprecated` and `Deprecated('1.12.15', 2032)`
in the definitions, which 93 other members carry. The generated C++ forwards the old name to the
new one and raises a `DeprecationWarning`; verified by setting it and reading the new name back.

**The scaling logic existed twice** — once in `EditDictionaryWithTypeInfo`, once in
`EditDictionary— ` and is one function now, `DialogScaling(root)`. That is 30 lines less to
keep in step, and one of the items RG6.2.7 would otherwise have had to clean up.

Two consequences worth recording, because both are the project's own machinery working:

- `parameterConversionTest` changed in **one line**: the dictionary of
  `VisualizationSettings.dialogs` lists `fontScaling` where it listed `fontScalingMacOS`, because
  a deprecated member is not part of `GetDictionary`. The reference was re-recorded, and the
  diff was read before it was accepted.
- the **stubtest baseline** gained `exudyn.VSettingsDialogs.fontScalingMacOS`. That is where
  every deprecated member is listed: the stub emitter skips them deliberately (a deprecated name
  is not part of the documented API) while the runtime still exposes them. The regeneration also
  dropped the stale `exudyn.misc.resultsMonitor` entry, which had been reported as "no longer
  occurs" for days.

**And it found a segfault.** `exu.VisualizationSettings().general.drawWorldBasis— ` a standalone
settings object, an existing deprecated member — kills the process with exit code 139. The
backlink those members forward through is set in one place only, for the settings of a
`SystemContainer`; a standalone object never gets it. My new member behaves exactly like the 93
others, so nothing here caused it, and it is RG4.4 (#2603) because the fix has to decide what a
copy of a settings structure means.

<a id="rg6-2-4"></a>
### RG6.2.4 — the value is edited where it stands (2026-09-23, #2604)

The one real rewrite of this group. The value used to be edited at the **bottom of the window**,
in an `Entry` and a `Combobox` that occupied the same grid cell and swapped by z-order
(`lower()`/`lift()`): a user selected a row at the top and typed at the bottom, and which of the
two widgets was in front depended on the type of the selected row. Both are gone, and four
handlers with them.

**The editor is placed over the value cell** of the selected row — `tree.bbox(item, 'value')`
gives the rectangle, `place()` puts the widget there. An `Entry` for a typed value, a `Combobox`
for `bool` and for every enum, committed on Return or when the focus leaves, taken away on
Escape. The double click that toggles a `bool` still does.

**The bottom row is now the line that sets the item**, in a read-only field with a **copy**
button beside it:

```
SC.visualizationSettings.general.textSize = 16.0
```

That is the maintainer's own request, and it is the same thing RG12.3 is to produce for a whole
settings structure: a dialog session that can be pasted into a script. The value is written as a
**Python literal** — a `String` and a `FileName` are quoted, an enum is prefixed with `exu.—
` and the prefix follows the structure being edited, so the same dialog opened on
`simulationSettings` writes `simulationSettings....`. Under the line stands the type, the size
where it is not scalar, and the description.

The validation is the one of #2597, unchanged: `CheckType` with the combo lists, then
`ConvertString2Value` for the range, and the error box names the path and the expected type.
`ItemPath()` builds the dotted path once, for the code line and for that message; the loop that
built it used to sit inside the click handler and was thrown away after each use.

**What was verified and what was not.** The module parses, the whole suite passes, the nine
checks pass — and nothing here can be seen without a window, which this session must not open
(rule 11). Whether a click now opens the editor too eagerly, whether the combo box reads well in
a cell, whether the copy button is where a hand expects it: that is the maintainer's to say, and
it is why this went over as soon as it built.

<a id="rg3-11"></a>
### RG3.11 — the C++ core section points into the documentation (2026-09-23, #2611)

Small and worth doing at once. *"The C++ core"* in *Advanced topics* is the section RG3.1 left
behind when the old C++ chapter was split into `docs/dev/ARCHITECTURE.md` and section 9 of
`docs/dev/CODING_STYLE.md`: three sentences and two pointers. The pointers were **GitHub URLs**,
written when those files were visible only to someone with a clone — and RG3.2 published the
developer documentation, so they were sending a reader out of the documentation to a raw file of
a page they were already inside of. They are relative links now, `../dev/README.md`,
`../dev/ARCHITECTURE.md`, `../dev/CODING_STYLE.md`, which the strict build resolves and which
work in the PDF as well.

The one sentence that was added is the maintainer's, and it is the opposite of a link: those
pages say what the C++ side is and how to work on it, and **that is as far as prose goes** —
for a deeper understanding of the core and for any low-level change it is inevitable to visit and
study the GitHub project itself, the sources, the generators that write parts of them and the
history that says why something is the way it is. A documentation that does not say where it
stops sends people looking for a page that was never written.

**One test came with it**, because raising the seven issues of this round broke it:
`testTheChangelogListsAResolvedIssueUnderTheVersionItProduced` resolved *the newest open issue*
of the store and expected to find it in the changelog. The newest open issue was #2610, an
**IDEA** — and the very next test is the one asserting that an IDEA never reaches the changelog.
The test now raises its own issue of a type the changelog carries. Nothing was wrong with the
tracker; a test that reads whatever the store happens to hold has no business asserting on the
type of it.

<a id="rg6-2-8"></a>
### RG6.2.8 — the line to copy looks like a line to copy (2026-09-23, #2605)

Four small things the maintainer asked for after using RG6.2.4, and one of them is not small.

**The code line sits in a box.** It was a borderless `Entry` in the dialog font on the window
background, which is a thing one reads, not a thing one copies. It is a `tk.Entry` with no border
inside a `tk.Frame` with `relief=tk.SOLID` and a background of its own now, in the **tree's font
one size smaller** — for which the widget class had to be told the `fontFactor` that
`EditDictionaryWithTypeInfo` computes with `DialogScaling(root)` and had kept to itself. The
fixed-width font went: the maintainer asked for the font of the cells.

**The label under it is gone**, and with it the description it repeated — the tooltip of
RG6.2.3 shows the same text, in full, where the mouse already is. The one fact it carried that
nothing else did was the **size** of a vector or matrix setting, and that moved into the tooltip:
`textColor [VectorFloat, size 4]`.

**`copy` is `copy line`**, which matters only because RG6.2.9 puts two more copy buttons beside
it.

**And three functions left the widget class**, which is the part that is not small:
`SettingsLeafList`, `ValueLiteral` and `SettingsPrefix` are module level now, they work on
dictionaries, and they open no window. `CodeLine()` is two calls. The reason is the test file:
everything that can be moved out of a tkinter class can be tested, and everything left inside it
can only be looked at by the maintainer. `test_guiValues.py` has four new tests because of it,
among them the one that matters for the copy feature — **every literal the dialog writes, for
every setting of both structures, is one Python reads back**. The walk over the settings tree
that the test file had written for itself is that same `SettingsLeafList` now, so it exists once.

One correction on the way: the example in the docstrings was
`SC.visualizationSettings.general.textSize`, which is a **deprecated** name (it moved to
`view0.window.globalFontSize` in 1.10.80) and therefore appears in no dictionary the dialog ever
shows. The examples name `openGL.lineWidth` now. A test asserting on a setting that does not exist
is a test that passes for the wrong reason, and it was a test that caught it.

<a id="rg6-2-9"></a>
### RG6.2.9 — what differs from the defaults, in colour and as code (2026-09-23, #2606)

**Changed means changed against the defaults** — the maintainer's decision, and the one that
makes the feature worth having: a dialog that marks only what *this session* touched tells a user
nothing about the model they opened. A row whose value differs from `exu.VisualizationSettings()`
is written in a dark blue and bold, **as the dialog opens**, and it stops being marked the moment
the value is typed back.

The defaults are one constructor call away, which had to be checked rather than assumed: a
settings structure Python builds on its own segfaults on any **deprecated** member (#2603, RG4.4)
— `GetDictionaryWithTypeInfo()` touches none, and returns all 470 leaves. The call is wrapped
all the same: if it ever fails, the marking stays off and the dialog does not.

**Two buttons, two windows**, because the maintainer asked for both notions after all:
*diff to default* and *this session*. Each opens a window that holds the changes **as the code
that makes them** — which is the second half of the request: a window that *shows* the changes
is also the one that copies them, so the "changed only" view of RG6.2.11 is struck out. Both go
through one comparison with a different reference dictionary:

```python
SC.visualizationSettings.openGL.lineWidth = 2.0
SC.visualizationSettings.general.graphicsUpdateInterval = 0.5
```

**The comparison is on the string the dialog shows**, not on the value. That is what makes a float
and an enum comparable at all — `0.1` read back from single precision is not `0.1— ` and it
marks exactly what a user sees in the cell and what the copied line writes. `SettingsCodeLines`,
`SettingsValueStrings` and `TreeLeaves` are the whole of it: the first two are module level and
work on dictionaries, the third hands the tree's own rows to them in the same shape, so **the
marking, the two windows and the tests are one piece of code**. Four tests came with it, including
the one that matters: a reference in which **every** value of `simulationSettings` differs, so
that every type is written as a line and every line has to parse as a Python assignment.

<a id="rg6-2-10"></a>
### RG6.2.10 — find a setting (2026-09-23, #2607)

Several hundred values in a tree of folders, and until today the only route to one was knowing
which folder it sits in. A **find bar above the tree**: CTRL-F (bound on the dialog window, so it
works wherever the focus is) puts the cursor in it, RETURN or **F3** or the *find* button steps to
the next hit and around at the end, and the **drop-down** beside it lists the hits so that one can
be picked instead of stepped to. A hit is jumped to, not filtered to: `tree.see()` opens the
folders the row sits in and scrolls it into view, and the focus stays in the find field so that
RETURN keeps stepping.

**Names first, descriptions second**, which is what the maintainer asked for and what
`FindMatches` returns: a hit in the **name** of the setting, then a hit anywhere in its **path**,
then a hit that is only in the **description** — and that last kind is labelled with the piece
of description that matched, because otherwise it looks like a hit for no reason. The function is
module level and works on the leaf list, so the four tests that check the order need no window;
one of them asserts that every path a search returns is a path the tree actually holds, since a
hit that cannot be jumped to is worse than no hit.

Two things were rejected, and they stand in the plan so that they are not proposed again:
**filtering the tree** to the hits (the tree is the map of where a setting lives, and filtering
takes the map away) and a **separate result window** (a third place to look, in a dialog that has
three already).

**And the manual says how the dialog works** —
{ref}`the visualization settings dialog <sec-overview-basics-visualizationsettings>` described
what the settings are for and how to resize the window, and nothing about editing in it. It now
says what RG6.2.3 to RG6.2.10 built: editing in the cell, the tooltip, the line to copy, the
colour of a value that differs from the default, the two windows, and the find. The **screenshot**
in that section is from before all of it and cannot be re-taken by a session that must not open a
window (rule 11) — it is the maintainer's to replace.

**A note for the reader of this log:** nothing in RG6.2.8 to RG6.2.10 has been seen. The module
parses, the whole suite passes, the tests of `test_guiValues.py` cover everything that
happens below the widgets — and whether a find bar of this shape is the one that helps, whether
the blue is readable on a row, and whether the two buttons are where a hand looks for them is the
maintainer's to say.

<a id="rg6-2-6"></a>
### RG6.2.6 — the key bindings come from one table (2026-09-23, #2591)

What the render window does when a key is pressed was written down **three times**:
`src/Graphics/GlfwClient.cpp` implements it, `docs/manual/GUI.md` tabulated it, and the help
dialog printed its own text. Two of the three were prose, and nothing kept them in step with the
first — so they had drifted, and the drift is the argument for this step:

- **four bindings that exist were documented nowhere**: **H** (which opens the help window
  itself), **R** (auto-rotate the model view), **CTRL+R** (raytracing on and off for the current
  view) and **CTRL+V** (open the window of the next configured view). **CTRL+7**, the 3D view, was
  in neither table either.
- the **keypad rotation keys were named wrongly in both copies**: they read *KEYPAD 2/8, 4/6,
  1/9*, and the renderer uses `GLFW_KEY_KP_7` and `GLFW_KEY_KP_9`. A user following either
  document pressed a key that does nothing.

`python/exudyn/misc/keyBindings.py` is the one table now: per binding the keys as a user presses
them, what it does, the remarks for the documentation, the short form for the help dialog, and
**the GLFW key names it is implemented with**. `RendererHelpText()` builds the dialog text from
it (the 53-line literal in `GUI.py` is gone), and `tools/generators/keyBindingsEmitter.py` writes
`docs/generated/mouseBindings.md` and `docs/generated/keyBindings.md`, which `docs/manual/GUI.md`
includes where its tables stood. Two prose copies became renderings of one source.

**The third copy cannot be generated, so it is compared.** The emitter reads the key tests out of
`GlfwClient.cpp` and reports what one side has and the other does not; today the two agree
exactly. `python/testing/test_keyBindings.py` makes that a test rather than a report: a
documented binding that nothing implements is a failure, and an implemented key that is documented
nowhere is a failure with a list of accepted exceptions that is **empty**. The next key someone
adds to the renderer will fail the suite until it is written down — which is the only way a
copy that cannot be generated stays honest.

<a id="rg6-2-7"></a>
### RG6.2.7 — GUI.py is cleaned up (2026-09-23, #2591)

Last, as the maintainer asked, because every other sub-step edited this module.

**Dead code out**: the `#EXAMPLE` dictionary at the end (19 lines of a call nobody makes) and 32
lines of commented-out code — `#print('select')`, `#print(kids)`, an abandoned `exec` string,
two font lines under the comment *"no effect"*. Kept, deliberately: the two commented lines that
say **why** the code around them looks as it does, such as the `float32` conversion that produces
the single precision the C++ side holds. A comment that explains is not dead code; a line of code
behind a `#` is.

**One way of reporting**: seven bare `print()` calls became `exudyn.Print`, with `WARNING:` or
`ERROR:` and the name of the function that failed — they go where the rest of Exudyn's output
goes, and a message such as *"showing of dictionary failed"* now says which dialog said it. The
command window keeps its plain `print`: that one is a transcript of what the user just ran, and
it belongs in the console.

**And the module stopped writing the user's configuration.** `treeEditOpenItems` is documented as
the list of folders a settings dialog opens with, and a user sets it — while the dialog
**appended to it and removed from it on every click on a folder**, so opening a folder once
rewrote the configuration for the rest of the process, and every `SystemContainer` shared the
result. The list is configuration now and nothing writes it; the dialog keeps its own
`self.openItems`, and what was open when a dialog was last used is remembered in
`treeEditLastOpenItems`, which says in its name that it is session state. The remembering that
the clicks used to do by accident is kept on purpose, and both lists are in the manual.

`GUI.py` is 1680 lines after all of RG6.2, having been 1017 when the group was written up —
against 528 lines that left `rendererPythonInterface.cpp` (RG6.2.1) and four dialogs, a tooltip,
a cell editor, the diff windows and the find that were added.

<a id="rg6-2"></a>
### RG6.2 is closed (2026-09-23, #2591)

Every sub-step is done or dropped. What the maintainer named on 2026-09-22 — a restricted
table, no type hints, no font scaling off macOS, columns that cannot be adjusted, descriptions
behind a special key, no inline editing, unhandy combo boxes — is answered by RG6.2.1 to
RG6.2.10, and **RG6.2.5 (a second front end) is dropped**: tkinter needs no installation, runs
everywhere and now does what was asked of it. What the group produced beyond the complaints is
the part worth remembering: the dialogs left the C++ (RG6.2.1), the layer under the widgets got
tests where there were none (RG6.2.2), the validator was made to agree with the settings it
validates (RG6.2.3), and everything that can be decided without a window is now a module level
function on dictionaries — which is why a group whose result cannot be seen by the session that
wrote it could be built at all.

Two things it leaves open, both deliberate: **RG6.2.11** (#2608), the catalogue of optional
features with **undo** at the top of it, and **RG12.3**, which is the *diff to default* of
RG6.2.9 for a whole settings structure rather than for one dialog.

<a id="rg6-2-12"></a>
### RG6.2.12 — the difference is measured against what a user starts from (2026-09-23, #2612)

The maintainer opened demo 2, pressed *diff to default*, and got **every light and every
material** in the list of changes — 59 settings nobody had touched. The reference was
`exu.VisualizationSettings()`, a settings structure Python builds on its own, and a
**`SystemContainer` initialises those 59 when it is created**: the four lights, and the ten
raytracer materials, which are synced with the renderer materials. The constructor's state is not
the state a user starts from, and nothing in this repository could have said so without creating
a `SystemContainer`: every test of RG6.2.9 built its reference the same way the defect did, so
they agreed with each other and with nothing else.

`DefaultSettingsDictionary()` takes the reference from `exu.SystemContainer().visualizationSettings`
now — a container opens no window, and it is the only way to the initialised state — and falls
back to the constructor for a structure that is not on a container, which is what
`simulationSettings` is. Three tests came with it, and the second one is the one worth keeping:
it asserts that the **constructor still differs**, and that every difference is a light or a
material. If that test ever fails, the initialisation moved into the structure itself and this
function can go.

**And a folded folder no longer hides a change.** The colour of RG6.2.9 marked the leaf, which is
invisible when its folder is closed — and the tree opens with four folders open out of some
sixty. `MarkChangedValues` walks post-order and marks a **folder** whose subtree holds a changed
value; after a single edit the folders above the row are recomputed instead of the whole tree.

<a id="rg6-2-13"></a>
### RG6.2.13 — the find bar loses its button (2026-09-23, #2613)

The search runs while the text is typed, so the **find** button was a button for something that
had already happened; it is gone. The drop-down of the hits is **disabled until there is a search
text**, because a combo box that is enabled and empty looks like a control that does nothing. Both
from the maintainer, after using it.

<a id="rg6-2-14"></a>
### RG6.2.14 — what a user does with a dialog (2026-09-23, #2614)

A **second row** at the bottom: *diff to default* and *this session* on the left, and on the right
the four things a dialog owes its user — **reset** (all settings to the defaults), **revert**
(to the state the dialog opened with), **undo** (the last change, one step, **greyed** until there
is one) and **close** (what ESCAPE does). Reset and revert ask first: they throw away everything
the model set, which is not a click's worth of consequence. The undo is the one from the
catalogue of RG6.2.11, and it is one step because that is what was asked for; every value the
dialog writes arms it, and a whole set written at once disarms it, since "back" would not be one
step any more.

**Every button says what it does**, in a tooltip, including the two that were there before —
*show diffs to default* and *show changes since dialog opened*, in the maintainer's own words.
The `Tooltip` class gained a `Bind()` for that, and a **delay of 0.5 seconds** for all of them:
a description that appears the moment the pointer crosses a row is in the way, and since RG6.2.10
nobody has to sweep the tree to find a setting.

**The window with the changes stayed behind the dialog** from the second time it was opened. The
dialog is topmost — it must be, it blocks the render window — and a plain `Toplevel` of a
topmost window is not. It is `transient` to the dialog and topmost itself now, lifted and
focused when it opens.

<a id="rg6-2-15"></a>
### RG6.2.15 — a folder says what it is (2026-09-23, #2615)

Every settings structure carries a `classDescription` in `definitions/— ` *"General settings
for visualization that influence all windows, default values, autofit, multithreading, etc."* —
and it reached the reference manual and the C++ header comment, but **not the dictionary the
dialog reads**: only leaves had a description there, so the pop-up had nothing to show over a
folder, which is where a user new to the settings looks first.

`structureHeaderEmitter.py` writes it into `GetDictionaryWithTypeInfo` under the reserved key
`structureDescription`, beside `itemIdentifier`, with the same guard against a settings member of
that name. `GetDictionary— ` the plain one, which round trips through `SetDictionary— ` is
untouched, so nothing a script does changes. The dialog stores it as the description of the
folder node and steps over the key when it builds the tree, and a test walks both structures and
requires that **every** folder of both has a non-empty description.

<a id="rg4-5"></a>
### RG4.5 — quitting is not failing (2026-09-23, #2616)

`python -m exudyn demo 2`, close the render window at *"Computation paused... press SPACE to
continue / Q to quit"*, and the script ends in a **traceback**: the *DYNAMIC SOLVER FAILED* block
and `exudyn.SolverError: SolveDynamic terminated`. Wait one step longer and press Q, and the same
quit ends the script quietly.

The asymmetry is one line. `CSolverBase::SolveSystem` begins with

```cpp
if (computationalSystem.GetPostProcessData()->forceQuitSimulation) { ...; return false; }
```

and `false` is what `SolveDynamic` and `SolveStatic` read as *the solver failed*: they print the
failure block and raise. The **mid-run** path never reaches that return — `SolveSteps` leaves
its loop when `stopSimulation` is set and returns `!conv.stepReductionFailed`, which is `true`,
so the script continues. It returns `true` now, with the note reworded to say that nothing was
computed. A user who quits is not a solver failure, and both paths say so.

**Two things this turned up that are worth knowing.**

`output.simulationStoppedByUser` cannot be set on this path: the solver never initializes, and the
`MainSolver` copy that Python reads is filled during initialization, so an assignment there reads
back as `false— ` which is worse than not offering it. The line is a comment naming
`mbs.GetRenderEngineStopFlag()` instead, and that is what a script asks.

**And it has no test**, which is the honest part. `forceQuitSimulation` is set in
`GlfwClient.cpp` and nowhere else, and it has no Python binding, so nothing but a real render
window can produce the state this fixes. `mbs.SetRenderEngineStopFlag(True)` sets the **other**
flag — `stopSimulation— ` which `InitializeSolver` clears when a solve starts; a test built on
it passes for the wrong reason, which is how it was found here. Two flags with one name in the
Python API is the deeper thing the maintainer suspected, and it stands in RG4.5 as the open half.

<a id="rg10-3"></a>
### RG10.3 — exudev says how long it took (2026-09-23, #2617)

The batch scripts `exudev` replaced printed the build time; the driver of revision2026 step R5.18
printed a verdict table and no time at all, and the maintainer misses it for a good reason — a
build that suddenly takes twice as long is the first sign that a header dependency grew, which is
exactly the question RG9 will ask about `pybind11`. Every step is timed, and the summary carries
the seconds and a total:

```
+++++ exudev summary +++++
  regenerate (venvExuP313)   ok           13.8s
  TOTAL                                   13.8s
```

Under 100 seconds it reads as `13.8s`, above it as `4m12s`, because nobody counts to 252.

<a id="rg10-4"></a>
### RG10.4 — the last file leaves src/pythonGenerator (2026-09-23, #2618)

`src/pythonGenerator/` held **one file**: `exudynVersion.py`, which finds the repository root and
reads `version.txt`. Everything else of the old generator directory moved to `tools/generators/`
in revision2026 step R4.3, and this one stayed because three very different things read it —
`setup.py` and `conf.py—exec()` it from the repository root, and `itemDocsEmitter.py` imports it
through a `sys.path` entry that `generatorPaths.py` added **for that single file**.

It lives in `tools/generators/` now, beside `createStubFiles.py` and `generatorPaths.py`, which
the build already needs and `MANIFEST.in` already ships; the `sys.path` entry is gone with it, and
so is the directory. Five files name the new path, and the ones that only remember the old one in
a comment — *"moved out of `src/pythonGenerator/pythonAutoGenerateObjects.py`"* — keep it,
because that is history and still true.

<a id="rg10-5"></a>
### RG10.5 — VS Code can follow an include (2026-09-23, #2619)

*"include errors detected - update your include paths"*, and, from `Main/CSystem.h`, *"cannot open
source file ../Eigen/Sparse"*. The reason is worth writing down, because it looks like a broken
include and is not one: the vendored headers are reached **through a subdirectory of `include/`**.
`src/Linalg/LinearSolver.h` says `#include "../Eigen/Sparse"`, and the compiler resolves that
against every include directory in turn, so it finds `include/lest/../Eigen/Sparse`. The build
passes `-Iinclude/lest -Iinclude/glfw -Iinclude/glfw/deps -Iinclude/pybind11local`; the C/C++
extension of VS Code knew none of them, so it could not follow a single include of the C++ sources.

`.vscode/` is git-ignored, so the fix follows the pattern the repository already has for
`exudyn.sln` and `python/pytest.py` (revision2026 step R2.x): a **committed template**,
`tools/vscodeCppPropertiesTemplate.json`, that `tools/setupLocalWorkspace.py` copies into
`.vscode/c_cpp_properties.json— ` creating the directory, which a fresh clone does not have. A
machine-specific edit therefore cannot be committed, and the template says why each path is in the
list. `${env:CONDA_PREFIX}/include` finds `Python.h` of whatever environment VS Code was started
from.

<a id="rg6-2-16"></a>
### RG6.2.16 — a window nobody could see (2026-09-23, #2621)

*"The buttons 'diff to default' and 'this session' show nothing."* They did exactly what they were
written to do, and that is the interesting part: a probe that builds the whole dialog **without
ever mapping a window** — `tk.Tk()` and its `Toplevel` withdrawn before anything is drawn, which
is how a session that must not open a window (rule 11) can still test one — found the two
changed settings, produced their lines and raised nothing. The window was there; it was **behind
the dialog**, at the same position, and the dialog is `-topmost` because it has to be.

Four things together make it visible, and none of them alone was enough: `transient` ties it to
the dialog, `-topmost` puts it in front of a topmost parent, an **offset** of 60 pixels means the
dialog cannot cover it even if the stacking fails, and `grab_set` makes it **modal**, which is
what no window manager puts behind. Modal is also the right behaviour: it is a window one reads,
copies from and closes.

Two more from the same message: *this session* is **changes since start**, which says what it
shows; and the rows at the top and the bottom span the **three columns of the tree**, not four, so
that the right-most button no longer sits under the vertical scroll bar.

**What this cost, and what it bought:** three rounds of "it does not work" for a feature whose
logic was right the first time. The probe is 40 lines and should have existed before RG6.2.9 —
it cannot see whether a window looks right, but it can prove that a handler runs, and that is
where two of the three defects of this evening were.

<a id="rg11-2"></a>
### RG11.2 — the demos stop writing into the current directory (2026-09-23, #2620)

`python -m exudyn demo 2` wrote `solution/chain.txt— ` relative to wherever it was started, which
in this repository is a directory beside the sources. It was untracked and unignored, and it was
staged by accident in this very session and caught only by reading the file list before the commit.

The demos write to **`tmp/solution/`** now, through one function that creates the directory when it
is missing, so a demo still works from any directory and leaves its files where this repository
already ignores them. `solution/` is in `.gitignore` as well, because an installed version of the
package still writes there and the next clone should not have to notice.

<a id="rg6-2-17"></a>
### RG6.2.17 — the dialog gave the renderer a different container (2026-09-23, #2623)

This is what RG6.2.16 was really about, and it is mine: **constructing an
`exudyn.SystemContainer()` replaces `exudyn.sys['currentRendererSystemContainer']`**. RG6.2.12
creates one to read the defaults — which was the right fix for the 59 false differences — so
from the moment the settings dialog opened, every part of the package that asks for *the
renderer's* container was handed the **throw-away** one:

- `ApplyDialogWindowSettings` read *alwaysTopmost* and *alphaTransparency* from a default settings
  object instead of the user's;
- `UpdateSettingsStructure` sent the **redraw signal** to it, so a settings change stopped
  reaching the renderer at all;
- and once the temporary was collected, reading a member of it was an **access violation**
  (*"no RTTI data"*), which is what made both change windows do nothing: the exception was raised
  before the window could be shown, and a tkinter command handler swallows it into the console.

The entry is saved and put back around the construction, and `GetRendererSystemContainer` now also
catches `RuntimeError— ` a container that is gone must not take a dialog down with it. A test
holds it: register a container, read the defaults, and require that the entry is **the same
object** and still usable.

**The lesson is the one about side effects at a distance.** The C++ side writes that entry in
`MainSystemContainer::AttachToRenderEngineInternal`, which is the documented place; that a plain
constructor also lands there is invisible from Python and was found only by printing object
identities. Anything in this package that creates a `SystemContainer` for a moment has the same
problem, and nothing warns about it.

<a id="rg6-2-11"></a>
### RG6.2.11 — the catalogue, decided (2026-09-23, #2608)

A list is only useful if someone goes through it, and the maintainer did, the same day:

- **reset** and **undo** were built into RG6.2.14, and the *"changed only" view* is what the two
  windows of RG6.2.9 are;
- **load and save to a file: no.** The code of RG6.2.9 is what a user keeps, and it belongs in the
  script rather than in a second format that nothing else reads;
- **units in the description: no**, and the reason is worth keeping: even a *position* has no unit
  Exudyn could name. The model's units are the user's implicit choice, so a unit in a description
  would be a guess printed as a fact — which is the same argument that keeps hand-counted
  numbers out of the documentation;
- **apply while it is open** was already the behaviour and stays;
- **the same dialog for `simulationSettings`** survives as RG6.2.18 (#2624), low priority;
- **remember the window** is undecided, and the step now says *how* it would work rather than only
  that it could. The geometry is one string; it could live in a module variable (this process
  only), in `visualizationSettings.dialogs` (travels with the model), or in a user configuration
  file. The danger is the **position**, not the size: a window remembered on a screen that is no
  longer attached opens where nobody can see it. The rule that makes it safe — restore the size
  always, restore the position only when the rectangle still lies inside the virtual desktop —
  is written down, so that the step, if it is ever taken, starts from it.

<a id="rg6-2-19"></a>
### RG6.2.19 — the dialog closed the render window (2026-09-23, #2625)

The worst defect of this group, and it is mine. *"Opening the visualizationsettings dialog closes
(crashes?) the render window."*

`MainSystemContainer()— ` the thing `exudyn.SystemContainer()` creates — calls
**`AttachToRenderEngineInternal()` in its constructor**, and its destructor calls `Reset()`, which
begins with **`visualizationSystems.DetachFromRenderEngine(...)`**. A `SystemContainer` that exists
for a moment therefore **takes the running render window away from the container that owns it and
hands it back to nothing**. RG6.2.12 created exactly such a container every time the settings
dialog opened, to read the defaults that a container initialises.

That also corrects RG6.2.17 (#2623) of the same day, which treated the symptom: restoring
`exudyn.sys['currentRendererSystemContainer']` put the Python entry back and could not put the
**render engine** back, because the damage is in C++ and happens on construction and destruction.
The restore is gone with the container that needed it; what stays from #2623 is
`GetRendererSystemContainer` catching `RuntimeError`, which is right on its own.

**The dialog creates no container**, and the reference is `exu.VisualizationSettings()` again —
with the 59 differences that made the maintainer report #2612 in the first place. The difference
now is that the window **says so**: *"the lights and the raytracer materials are initialised by the
SystemContainer, so they appear here even when nothing touched them"*. An honest note is better
than a number that is wrong, and RG6.2.20 (#2626) removes the need for both by moving those
values into the definitions, where `exu.VisualizationSettings()` can see them.

**Two tests hold the line.** One greps the module for `exudyn.SystemContainer()` and fails if it
ever comes back — a crude test, and the only kind that can catch this without a render window.
The other requires every difference between the plain defaults and a fresh container to be one of
the paths the module lists, so the note in the dialog cannot quietly become untrue.

**And the dialog steps aside.** The window with the changes kept coming up *behind* the settings
dialog, which keeps itself `-topmost` because it blocks the render window. The maintainer's own
suggestion was to test without that flag, and that is what happens now: the flag is taken off the
dialog and off its root while the window is open, and put back when it closes.

**What this session should have done differently:** the fix of #2612 was verified against the
settings it produced, and never against *the process it runs in*. A `SystemContainer` looked like
a value, and it is a handle on the render engine. Nothing in Python says so — the constructor
that attaches is fifteen lines of C++ in a header — but the question *"what does this object do
to the session when it dies?"* is one to ask before creating one inside a running renderer.

<a id="rg9-1"></a>
### RG9.1 — the item sources stop paying for pybind11 (2026-09-23, #2622)

The maintainer's question was concrete: every `C<Item>.cpp` that draws something includes
`Graphics/VisualizationItemHelpers.h`, and that header pulled in pybind11 — *"could one omit the
pybind11 dependency easily?"*. The answer is yes, in four moves, and the measurement at the end is
not the one that was expected.

**What was in the way.** `VisualizationItemHelpers.h` includes `VisualizationSystemContainer.h`,
which had two reasons to reach pybind11: six free `Py...BodyGraphicsData...` declarations of its
own, and `#include "Main/CSystem.h"— ` a line the maintainer had marked *"REMOVE: temporary"*
years ago — which arrives at pybind11 through `Pymodules/PythonUserFunctions.h`.

**What was done.**

1. `Graphics/VisualizationSystem.h` includes `Main/CSystemData.h` and `Graphics/PostProcessData.h`
   itself. It declares members of both types and had no includes at all, free-riding on whoever
   included it. The order matters: `PostProcessData.h` includes nothing and uses `CSystemState`,
   so `CSystemData.h` has to come first — which the build said, not a reading of the code.
2. The six declarations moved to the new `Graphics/BodyGraphicsDataPython.h`, and with them the
   four pybind11 includes and the `namespace py` alias.
3. `VisualizationSystemContainer.h` dropped `Main/CSystem.h`.
4. Two generated headers had been free-riding too, and that is what the build found:
   - the four `VisuObject*.h` with a `graphicsDataUserFunction` name `py::object` in the member
     type. They now include `Pymodules/PythonUserFunctions.h`, which only **forward declares**
     `pybind11::object— ` so this costs no pybind11 either;
   - the eight `MainObject*.h` with a `BodyGraphicsData` parameter **call** the six functions, so
     they include the new header. `itemHeaderEmitter.py` emits both, from two flags read off the
     member list; twelve generated files changed and nothing was edited by hand.

**The measurement, and it is not the one the step expected.** The step said acceptance is the
build time. **It did not move**: 57.1 s before, 58.0 s after, clean builds, same machine —
within the noise of two runs. What did move is the dependency itself: of the 52 sources in
`src/ImplObjects/`, **52 reached pybind11 before and 19 after**. The 19 reach it for reasons of
their own — a user function, `Pymodules/PyMatrixContainer.h`, `pybind11/numpy.h` in the FFRF
objects, `Utilities/ExceptionsTemplates.h— ` and not one of them through the graphics headers.
That pybind11 is still compiled 19 times, and that the remaining sources share most of their other
headers anyway, is the honest explanation for the clock.

So the step delivered the structure and not the speed, and the plan now says so rather than
claiming a saving. Whether the *next* 19 are worth chasing — `ExceptionsTemplates.h` alone
accounts for eight of them — is a question for RG9, with the same measurement attached.

**A test holds it.** `python/testing/test_cppIncludes.py` walks the include graph of the
repository and requires the three graphics headers to be free of pybind11, and the count of item
sources that reach it to stay at or below 19. It is a structure test: it says nothing about what
the code does, only about what a compiler has to read, which is exactly the property that decayed
unnoticed for years.

<a id="rg6-2-21"></a>
### RG6.2.21 — the dialog stopped asking, and undo means undo (2026-09-23, #2627)

Two small things from the maintainer using the second button row, and the second one is a design
correction that makes the feature honest.

**The questions are gone.** *"The reset button now has a 'reset all settings to their default
values' question — I did not ask for it and it is not needed: we can always revert to initial
settings and there is undo. So no worries about one wrong button click."* Right, and the same for
revert. A confirmation that is always answered with yes teaches people to click through
confirmations; the way to make a destructive button safe is to make it reversible, which these two
already are.

**And undo now means undo.** It went back one value and was *disabled* by reset and by revert —
the two clicks a user would most want to take back. *"I thought that undo reverts to the previous
state — this would then always work."* That is the fix, and it is simpler than what it replaces:
instead of remembering one `(row, value before)` pair, the dialog pushes the **whole state** onto
a stack before every change, and undo pops one. A state is the ~470 value strings the tree shows,
which costs nothing beside the redraw each change triggers, and one implementation now serves a
single edit, a `bool` toggle, a reset and a revert. `ApplyValues` grew one honest argument,
`pushUndo`, which is false only for the undo itself.

Measured with a **withdrawn** Tk root, which maps no window: open, edit one value, reset, then
undo twice — `3.0 -> 9.0 -> 1.0 -> 9.0 -> 3.0`, with the stack at 0, 1, 2, 1, 0 and the button
disabling itself when it empties.

**A note on the probe, because it cost something.** The first run of it called `OnReset` against
the **installed** package rather than the source, hit the old code, and a `tk.messagebox` opened
— a window on the maintainer's screen, which CLAUDE.md rule 11 exists to prevent. The
environment variables do not cover tkinter message boxes, only Exudyn's own windows. What a probe
of a dialog must do is check that it is testing the code it just changed, and stay away from any
handler that can open a modal box.

<a id="rg6-2-20"></a>
### RG6.2.20 — the defaults of the lights and the materials come out of C++ (2026-09-23, #2626)

The maintainer's diagnosis was the one that mattered, and it is sharper than the step was written:

> *"the 'diff to default' does not work ... because it takes the default values for lights and
> materials as they are defined in the default VSettingsMaterial, but this does not make sense —
> there is a 'default' material0, material1, etc. which are defined later. Same for lights ...
> This is more like a bug and we need a solid fix, in order to represent that also in the docu."*

So the settings dialog was not merely noisy. It compared against a state that **never exists**:
`VisualizationSystemContainer()` dimmed `light1` to `light3` and turned two of them off after
construction, and `MainGraphicsMaterialList::Reset()` filled the ten raytracer materials, all of
it **after** the structure had been built. The generated reference had the same problem from the
other side — it printed the defaults of `VSettingsLight` and `VSettingsMaterial`, which is not
what `light1.diffuse` or `material1.baseColor` start from, and the class description carried an
apology for it: *"the default values shown in the documentation only reflect material0 but not
all 10 default materials"*.

**The fix is in the definition language, not in the dialog.** `StructureParameter` gained
`memberDefaults`: for a member whose type is another structure, *this* instance's own starting
values, written exactly as that sub-member's `defaultValue` would be. The 89 values — 80 for
the materials, 9 for the lights — are now in
`definitions/structureDefsVisualizationSettings.py`, `structureHeaderEmitter.py` writes them into
the generated constructor, and the C++ that set them afterwards is gone:
`MainGraphicsMaterialList::Reset()` copies from a fresh `VSettingsRaytracer`, which is also what
keeps the Python-facing `SC.renderer.materials.Reset()` working.

**And the documentation says it**, which is what the maintainer asked for. `structureDocsEmitter.py`
appends the instance's own values to its row:

> `light2— ` *settings for light2 and shadow; starts from diffuse=0.2, specular=0.2,
> enable=False; every other value is the default of the type*

The apology in the class description is gone with the reason for it.

**Every value was checked, not transcribed and hoped for.** A script read the 89 assignments out
of the C++ at `HEAD`, parsed them, and compared them with what a fresh `SystemContainer` reports:
**all identical**. That is the only way to move 80 hand-written numbers with a straight face.

**What this un-blocks.** `containerInitialisedSettings` in `exudyn.misc.GUI` is **empty**, the
note that RG6.2.19 had to put into the diff window is gone, and the test of that step now requires
the difference between a container and a plain `exu.VisualizationSettings()` to be **empty** —
the same test, with the assertion turned around, which is why it was written that way.
`test_settingsDefaults.py` pins the ten material names and the marks that tell them apart, so an
edit to those values is a decision rather than an accident.

<a id="rg6-4"></a>
### RG6.4 — the lights say what is true (2026-09-23, #2609)

The descriptions in `definitions/structureDefsVisualizationSettings.py` are the documentation of
the lights: the settings dialog shows them, the reference manual prints them, an editor completes
them. Three faults, all the maintainer's, and the second is the one that misleads.

**"of GL_LIGHT[0,1,2,3]" is gone.** Six members of `VSettingsLight` repeated it, which inside
`light0` says nothing a reader can use. They read *"of this light"* now, and the mapping —
`light0` to `light3` are OpenGL's `GL_LIGHT0` to `GL_LIGHT3— ` is stated **once**, at `enable`,
where a reader meets the light first.

**Every light casts a shadow.** `position` claimed *"light0 is also used for shadows, so you need
to adjust this position"*, which was true when only one light could, and is a performance decision
that no longer holds. The sentence is gone; `shadow` now says that every light can cast one and
that the effects accumulate — which the raytracer half of the same description had said all
along, so the page contradicted itself.

**And no generated text quotes a number no generator produced.** *"approximates directional lights
by enlarging the direction to 200 times maxSceneSize"* sat in `position`, where it described
shadows rather than a light, and it named a factor that has already changed once. It is in
`shadow` now, as *"a multiple of maxSceneSize"*. A documented constant that lives only in a
description is wrong the day someone tunes it, and nothing tells them.

**Two names in the hand-written manual** went with them: `introductionBasics.md` still recommended
`openGL.enableLight1` and `openGL.light0position`, both deprecated — the reader who copies them
gets a deprecation warning from the settings it tells them to use.

<a id="rg4-4"></a>
### RG4.4 — two lines of Python no longer segfault (2026-09-23, #2603)

```python
import exudyn as exu
exu.VisualizationSettings().general.drawWorldBasis     #exit code 139
```

Not that member: **all 93 deprecated members**, on read and on write. Each forwards to its
replacement through `backlink->view0.scene.drawWorldBasis`, and the backlinks are set by
`VisualizationSettings::Init(&settings)`, which was called in **exactly one place** — for the
settings that belong to a `SystemContainer`. A structure Python constructs never got it, so every
backlink was `nullptr` and the first deprecated access dereferenced it.

**The top class links itself.** `Init(this)` in the default constructor is the one line, and the
step was right that it is not the whole answer: the implicit copy constructor copies the backlink
of every sub-structure as well, so a copy would point at the **original** and writing a deprecated
member of the copy would change the original's replacement. The generated top class therefore
defines a copy constructor and a copy assignment that copy the members and then call `Init(this)`.
That is the answer the step asked for, and it arrives before RG12.1 gives `simulationSettings`
deprecated members for the first time: whatever class is the top of its file gets the same code
from the same emitter.

**And a missing link is now a sentence, not a crash.** Every deprecated forwarding starts with

```cpp
if (backlink == nullptr) { CHECKandTHROWstring("general.drawWorldBasis is deprecated and forwards
    to view0.scene.drawWorldBasis, which needs the settings structure it belongs to; this one was
    constructed on its own and is not linked"); }
```

which is what a standalone **sub**-structure still hits — `exu.VSettingsGeneral()` is bound and
has no parent to link to. A guard is cheap and a segfault is not a diagnosis; the next missing
`Init`, wherever it comes from, will say so.

**The test is crude on purpose.** A segfault cannot be caught, so the only way to find one is to
touch everything: `test_settingsBacklinks.py` reads **every** member of every sub-structure of a
standalone structure and of a container's, counts how many raised a `DeprecationWarning— ` 314
members read, **93** deprecated, zero problems — and fails if that count falls far enough that
the walk would stop proving anything. If a future change brings the crash back, the test process
dies, which is the report.

**What is not tested**, and said rather than hidden: the copy constructor and the copy assignment
cannot be reached from Python, because pybind exposes neither. What the tests show is that two
standalone structures are independent. The copy operations matter on the C++ side, where a
settings structure is assigned, and the generated code is the guarantee there.

<a id="rg9-2"></a>
### RG9.2 — an include fourteen sources did not use (2026-09-23, #2628)

RG9.1 left a clean target: after the graphics headers stopped carrying pybind11, **19** of the 52
sources in `src/ImplObjects/` still reached it, and `Utilities/ExceptionsTemplates.h` was the only
route for **eight** of them. The step that was proposed for it assumed a refactor. The measurement
said otherwise.

**The header is included by 17 item sources and used by one.** `CObjectANCFBeam.cpp:45` calls
`GenericExceptionHandling`; `CObjectFFRF.cpp:423` and `CObjectFFRFreducedOrder.cpp:459` have their
call commented out. The other fourteen refer to nothing in it at all — not a template, not
`SetPendingExceptionCause`, nothing — and the header defines no macros that could hide a use.
So the whole step is one deleted line per file, and the eight sources whose only route to pybind11
it was now compile without it: **19 to 11**.

**The build then found what the include had been hiding**, which is the part worth remembering.
The generated header of every item with a user function — **24** of them, objects, loads and
`CSensorUserFunction— ` names `py::object` in the type of that member and had been taking the
`namespace py` alias from whatever happened to be included before it. Two failed the build the
moment their source stopped including the exceptions header (`CObjectGenericODE1.h`,
`CObjectConnectorCoordinateVector.h`); the other 22 were one include away from the same. That
is the same defect RG9.1 found in the four `VisuObject*.h`, and it has the same fix:
`itemHeaderEmitter.py` emits the alias beside the `Pymodules/PythonUserFunctions.h` it already
emitted, and that header only **forward declares** `pybind11::object`, so the alias costs nothing.

An unused include is not only waste: it is a **load-bearing accident**. Nobody wrote
`CObjectGenericODE1.h` to depend on an exceptions header, and nobody could have known it did.

**And a second latent defect fell out of the regeneration** (#2629). `itemHeaderEmitter.py`
reads an existing generated header with `encoding='utf8'` and wrote it with a bare
`open(fileName,'w')— ` the **locale** encoding, cp1252 on this machine. It could only bite when
a header containing a non-ASCII character was rewritten, and two do: `CObjectFFRF.h` and
`CObjectFFRFreducedOrder.h` carry *Zwölfer Andreas* in the author line. The moment this step
changed every user-function header, those two were written as cp1252 and the **next** run of the
generator died reading them — `UnicodeDecodeError`, byte `0xf6`. The write gets `encoding='utf8'`.
A bug that needs two unrelated conditions to meet is exactly the kind that waits years, and this
one waited for a step that touched every header at once.

**The build time did not move again**: 56.4 s, against 58.0 s after RG9.1 and 57.1 s before it.
Three measurements, one conclusion — on this machine the item sources are not what the compiler
spends its minute on. The value of both steps is that the dependency is now what it claims to be,
and `test_cppIncludes.py` holds the count at 11 so it cannot quietly climb back.

<a id="rg6-2-22"></a>
### RG6.2.22 — the bool toggle came back (2026-09-23, #2630)

*"Previously, bool variables had the feature that a double click switched the state — please
restore that feature."*

The toggle was never removed. RG6.2.4 (#2604) gave the tree an editor **in the cell**, bound to
`<ButtonRelease-1>`, and for a `bool` that editor is a Combobox **placed over the value cell**. So
the first click of a double click put a widget under the mouse, the second click went to that
widget, and the `<Double-1>` binding on the tree never fired. Fifteen lines of working toggle code
sat there unreachable.

The fix is the ordinary one for this conflict, kept as narrow as it can be: **on a bool row the
cell edit is scheduled** with `after(220 ms)` instead of opened at once, and a double click cancels
the job before it fires. Every other type keeps the editor that opens immediately, so nothing else
got slower — a float still edits on the first click.

Checked with a withdrawn Tk root: a single click on `nodes.show` leaves the value alone and arms
the job; the double click toggles `True -> False`, and again `-> True`; each is its own undo step;
a float row schedules nothing and has its editor up at once.

**What this says about the earlier step:** RG6.2.4 was verified by editing values, which is what it
was for, and a *regression* in a feature it did not mention went unnoticed for a week. A bound
event that another binding can swallow is not visible in either piece of code.

<a id="rg6-2-23"></a>
### RG6.2.23 — a font scaling that actually scales (2026-09-23, #2631)

*"dialogs.fontScaling does not work at all. Using 1.0 gives a larger font, but much too small row
height and smaller column width. Changing fontsize does not resolve that — only 0.0 works."*

Right, and the numbers say why. `DialogScaling` set `systemScaling = fontScaling` when the setting
is greater than zero, and **systemScaling was what the layout was computed from**:

- the row height was `int(treeviewDefaultFontSize * textHeightFactor * systemScaling)— ` at
  `fontScaling=1` that is `int(9*1.45*1) = 13` pixels, while the font on this display really draws
  with a **linespace of 16 to 18**, because the point-to-pixel conversion follows the tk scaling
  that `GetGUIContentScaling` had set and `fontScaling` never touches. Hence rows too small for
  their own text;
- the column factor was `max(1, int(round(systemScaling)))— ` an **integer**. It is 1 for every
  `fontScaling` below 1.5, so the columns could not widen at all while the glyphs did.

**So the layout stops guessing and measures.** `DialogRowMetrics(root, fontFactor)` builds the font
the tree will use and asks it: the row height is its `metrics('linespace')` times `rowHeightFactor`
(1.15, chosen to reproduce the pixel height the dialog had at the default font), and the column
scale is the width of `'0123456789'` in that font divided by the width in the unscaled one. Both
are then right at any display scaling, on any platform, for any value of the setting:

| fontScaling | font | row height | columns | row height before |
|---|---|---|---|---|
| 1.0 | 9 | 17 | 1.00 | 13 |
| 1.25 | 11 | 20 | 1.14 | 16 |
| 1.5 | 14 | 25 | 1.57 | 19 |
| 2.0 | 18 | 31 | 1.86 | 26 |

The default appearance does not change: at `fontScaling=0` the column scale is exactly 1 and the
row height is within a pixel of what it was.

**Confirmed by the maintainer** at 0.5, 1 and 2, with one request that came out of seeing it
work: the name, value and type columns are **25% wider** than they were (325, 188 and
113 pixels at the unscaled font). The description keeps its width and still takes the
rest of the window.

**The lesson is about the name.** `systemScaling` was one number doing two jobs — how large the
font is *and* how much room the layout needs — and they are only the same number on a display
whose scaling is 1. A value that means two things is wrong in one of them as soon as they diverge,
and this one had diverged since the setting was added.

<a id="rg12-3"></a>
### RG12.3 — what did this model actually change? (2026-09-24, #2590)

The settings dialog could always show it for one session. A **script** could not, and the reason
was an import: everything needed sat in `exudyn.misc.GUI`, which does `import tkinter` on line 14,
at module scope, because the dialog class inherits from `tk.Frame`. A model on a machine without
tkinter — a cluster, a container, a CI runner — could not ask what it had changed.

**So the window-free half moved out**, into `exudyn.misc.settingsUtilities`. On the name: the
maintainer asked whether `settings.py` was right for a file that only operates on settings. The
package already answers it — `basicUtilities`, `advancedUtilities`, `rigidBodyUtilities`,
`graphicsDataUtilities— ` so `settingsUtilities` it is: it says *operates on* settings rather
than *is* settings, which is what `visualizationSettings` and `simulationSettings` are.

What moved: the type predicates (`IsFloat`, `IsArrayInt`, `IsVector`), the conversions
(`ConvertString2Value`, `ConvertValue2String`, `CheckType`, `GetComboBoxListsDict`) and the
settings layer of RG6.2.8 to RG6.2.10 (`SettingsLeafList`, `ValueLiteral`, `SettingsCodeLines`,
`SettingsValueStrings`, `DefaultSettingsDictionary`, `FindMatches`, `SettingsPrefix`). 322 lines,
and the compiler of this move was **ruff**: `--select F821` found `IsFloat`, which the block used
and which had stayed behind.

**The three new functions** are thin, because the machinery was already there and tested:

```python
from exudyn.misc.settingsUtilities import PrintChangedSettings
PrintChangedSettings(SC.visualizationSettings)
#3 settings of SC.visualizationSettings differ from the defaults
#SC.visualizationSettings.nodes.show = False
#SC.visualizationSettings.openGL.lineWidth = 2.0
#SC.visualizationSettings.general.textColor = [1.0, 0.0, 0.0, 1.0]
```

`ChangedSettings` returns `(path, line)` pairs, `ChangedSettingsCode` the pastable block, and
`PrintChangedSettings` prints it. A `reference` argument turns *"differs from the defaults"* into
*"changed since this moment"*, which is what the dialog calls **changes since start** — the same
function, without a window.

**The test that matters is the crudest one**: it starts a fresh interpreter, imports the module and
requires `'tkinter' in sys.modules` to be **False**. Every other test here would keep passing if
that broke, and it is the one property the whole step exists for. The second is the round trip:
the generated block is `exec`-ed against a fresh structure, and the result must equal the one it
was read from — a pastable line that does not reproduce its setting is worse than none.

**The names stay importable from `exudyn.misc.GUI`.** They were its public API; `checkAll` keeps
an imported name out of `__all__`, so each is documented once, on the page of the module that
defines it, and nothing that used them breaks.

**Solution and sensor files, which the step raised as worth considering: no C++ change.** The
ASCII solution header is written by `CSolverBase::WriteSolutionFileHeader` and the binary one is a
fixed field sequence with its own version tag — but a free-text escape hatch already exists and
is already in the header: `simulationSettings.solutionSettings.solutionInformation`. So
`solutionSettings.solutionInformation = ChangedSettingsCode(simulationSettings)` makes a solution
file reproducible today, and the function's docstring says so. Adding a second mechanism beside it
would have been the wrong kind of completeness.

<a id="rg6-5"></a>
### RG6.5 — the renderer restores its own saved state (2026-09-24, #2633)

`SC.renderer.Stop()` has always saved the render state of every open view into `exudyn.sys`, and
every model that wanted the previous view back said so in two lines:

```python
if 'renderState' in exu.sys:
    SC.renderer.SetState(exu.sys['renderState'])
```

**82 occurrences in 85 files**, and the survey is the reason the replacement could be scripted at
all, because they were not 82 copies of one line. Five variants: 26 one-liners, 46 two-line forms,
12 with a trailing comment, 2 in the shipped package spelling the module `exudyn` rather than
`exu`, and **4 still using the deprecated `SC.SetRenderState`**. Within those, the indentation is
0, 4, 8 or 12 spaces, the brackets are written `['renderState']` and `[ 'renderState' ]`, and
`python/Examples/multiMbsTest.py:105` calls it on **`SC2`** rather than `SC`. A `sed` would have
got three of those wrong.

**The function is C++ because the dictionary is C++.** `exudyn.sys` is a `py::dict` held by the
module, written by `PyStopOpenGLRenderer`; `MainRenderer::RestoreSavedState(viewID)` reads the same
key the saving code writes — `renderState` for the main view, `renderState<N>` for the others —
and hands it to the existing `SetState`. **Nothing saved is not an error**: it returns `false` and
changes nothing, which is exactly what the `if` in those two lines was doing.

The binding is generated: the entry goes into `definitions/pybindRenderer.py`, and one
regeneration writes the pybind header, the `.pyi` stub and `docs/generated/cInterface/Renderer.md`.

**Two things the script got wrong, and both were caught by a check rather than by reading.**

- `python/Examples/mouseInteractionExample.py` had the guard with an **`else:`** — a hard-coded
  view for the first run — and collapsing the `if` orphaned the `else`. `checkExtras` reported
  the file as unparsable, which is how it was found. It reads better now than before, because the
  return value is what the `else` needs: `if not SC.renderer.RestoreSavedState():`. The same file
  also had `SC.renderer.SetState(renderState)` written twice in a row; one is gone.
- three files came out with `CR CR LF` on every line, from a line-ending conversion applied twice.
  A byte check found it; the same slip had happened in RG12.3 the day before, which is twice for
  one mistake and the reason the replacement script now ends with an `ast.parse` of every file it
  wrote.

**A measurement corrected an assumption in the plan**: it said a bad value in the dictionary would
be reported by `SysError` and not raise. It raises. `RestoreSavedState` adds no error handling of
its own and passes that through, and the test says so rather than asserting the comfortable thing.

`python/testing/test_rendererState.py` covers the four cases that matter — nothing saved, a state
that comes back, an unknown key ignored, a bad value raising — and one crude one: no source file
under `python/` or `docs/manual/` may contain the old idiom, so the sweep cannot be half done.

<a id="rg10-6-1"></a>
### RG10.6.1 and RG10.6.2 — the channel, and the first model (2026-09-24, #2632)

The runners now talk to a model through `exu.sys` instead of an import. Before each model they set
`testIsActive`, clear `testResult— ` it outlives a model, so a value left from the previous one
would be read as this one's result — and honour an `exu.sys['testTolerance']` the model may set
for itself. `exudynTestGlobals` still works beside it, so a model that has not been converted runs
unchanged.

`bricardMechanism.py` is the first model: nine lines and an import became

```python
testIsActive = exu.sys.get('testIsActive', False)
```

the hard-coded reference `4.172189649307425` is gone (`runTestSuiteRefSol.py` has held it all
along), the local variable that held the **result** and was called `testError` has the right name,
and the file ends with `exu.sys['testResult'] = testResult`. The number is **identical**:
4.172189649306508.

**And a window opened on the maintainer's screen**, which is the part worth recording. The plan
said, as a measurement, that the `if useGraphics:` around the renderer calls guarded nothing
because the runners suppress windows. That was measured on the **worker bootstraps**
(`testRunnerTools.py:724,886`), which do call `SuppressAll(True)— ` and not on the **serial**
path of `runTestSuite.py`, which did not. The models' own branch was the only thing keeping windows
shut there. The fix belongs in the runner and not in 125 models, so `runTestSuite.py` calls
`SuppressAll(True)` like the workers; and the maintainer's decision is that the models **keep** an
explicit `if not testIsActive:` anyway, because a test is worth running with and without graphics.
A measurement of half the paths is not a measurement.

<a id="rg10-6-3"></a>
### RG10.6.3 — 129 test models stop importing the test suite (2026-09-24, #2632)

Every model carried the same nine lines and an import of `modelUnitTests`. They are one line now:

```python
testIsActive = exu.sys.get('testIsActive', False)
```

**The survey came before the script**, and it is what made the sweep safe. The header block turned
out to be uniform — 124 files, byte for byte the same `useGraphics = exudynTestGlobals.useGraphics`
— but everything around it was not:

- the standalone default is `useGraphics = True` in 122 files and **False** in two
  (`symbolicModuleTest`, `symbolicUserFunctionTest`). Since `useGraphics` is exactly
  `not testIsActive`, the converted line keeps each file's own default:
  `exu.sys.get('testIsActive', False)` for the first, `..., True)` for the second, so standalone
  behaviour is unchanged in all of them;
- **16 models re-assign the flag** after the header (`useGraphics = False` to force the graphics
  off for that model); those became `testIsActive = True`, which is the same statement;
- `useGraphics` appears in expressions, not only in `if`: `0.3*useGraphics`,
  `2 if useGraphics else 0`, `(1-useGraphics)`, `useGraphics+1`, `writeToFile = useGraphics`. The
  script writes `(not testIsActive)` there, parenthesised, and the readable `if not testIsActive:`
  where it is a plain condition;
- `exudynTestGlobals.performTests` (2 uses) is the same flag under another name;
- `testResult` is not always a plain assignment: seven `+=` and one `*=`. Every one of them has an
  initialising `= 0` above it, which is why a mechanical rename was safe;
- **`ACFtest.py` had its own `useGraphics=True`** and never used the suite at all, and
  `computeODE2EigenvaluesTest.py` carried the whole block while **nothing in it ever read the
  flag** — that one simply lost the block.

**The script refuses rather than guesses.** A file whose header it does not recognise, that still
mentions `exudynTestGlobals` afterwards, where `useGraphics` survives, or whose result does not
`ast.parse`, is left untouched and reported. It reported four files on the first batch and three
more later; each was read and handled, and one of those reports found the two files above.

**What it got wrong, and what caught it**: a commented-out switch, `#useGraphics = False`, became
`#(not testIsActive) = False`, which is not Python even in a comment. 30 of them in 27 files, put
right afterwards — a replacement on an identifier does not know it is inside a comment, and a
comment is what a reader copies when they want to turn the graphics on.

**The bar was identical numbers, not a passing suite.** The results of all 139 models were captured
before the sweep and compared after every batch. One difference appeared and was **not** the sweep:
`taskmanagerTest.py` changes in its last digit from run to run, which two runs of the unchanged
tree confirmed. At the end, **all 139 are identical**.

**What went out with the boilerplate**: 98 lines of the form
`exudynTestGlobals.testError = result - (4.172189649307425)`, a second copy of a reference solution
that `runTestSuiteRefSol.py` has held all along, and the prints of that difference. The suite
computes the error itself and always did.

One thing is deliberately left: `kinematicTreeAndMBStest.py` multiplies its result by `1e-7` to
make it fit the tolerance. That is the pattern the maintainer asked to stop, and removing it moves
the value by seven orders of magnitude and with it the reference — so it becomes
`exu.sys['testTolerance']` in RG10.6.7, where changing that number is the point rather than a side
effect.

<a id="rg10-6-5"></a>
### RG10.6.4 and RG10.6.5 — modelUnitTests.py is gone (2026-09-24, #2632)

**The mini examples are generated**, which is the whole of RG10.6.4: the three lines that made
each of the 24 files import the test suite are in `tools/generators/miniExampleEmitter.py`, and
the `exudynTestGlobals.testResult = ...` at the end of each is in the 24 `miniExample` bodies in
`definitions/`. Both changed; one regeneration rewrote the 24 files. A mini example now imports
`exudyn` and nothing else, and it runs from any directory, which the two `sys.path.append('../testing')`
lines existed to work around.

**And modelUnitTests.py is deleted**, with `runUnitTests.py`. What it held:

- **ten test functions**, each taking `(mbs, testInterface)` and returning an error. They are ten
  ordinary files in `python/TestModels/` now. The extraction was scripted — dedent the body,
  `testInterface.SC` to `SC`, the single top-level `return X` to `testResult = X` plus a print and
  `exu.sys['testResult']— ` and the script refused a function it could not take apart cleanly
  rather than guessing;
- **`TestInterface`** and **`RunAllModelUnitTests`**, gone with them;
- **`ExudynTestStructure`**, which is *not* gone: `testRunnerTools.AddTiming` collects a `timings`
  list on it for the seven performance models (#2460). It moved to `testRunnerTools.py`, where the
  rest of the runner machinery lives, and the performance models import it from there. RG10.6.8
  finishes that.

**The finding that makes this worth more than tidiness**: `runTestSuite.py` had
`TSScope.runUnitTests = False #skipped at least since V1.6`. **The ten tests were not being run.**
Dead since 2021, and one of them — `GraphicsDataTest— ` contains
`testInterface.testinterface.SC.renderer.Start()`, a typo that would have raised the moment
anybody turned the switch on. They run now, and all ten pass.

**They also made the tolerance feature real.** `RunAllModelUnitTests` compared their errors
against `errTol = 4e-13`, which is looser than the suite's 5e-14 — and `SliderCrank2DTest`
really does produce 6.0e-14. So each of the ten states
`exu.sys['testTolerance'] = 4e-13`, the thing RG10.6.1 built, and their reference solution is
**0**, because what these old tests compute *is* an error against a value written into them in
2019.

That flushed out two runners that did not know about the feature yet: the **pytest** runner
(`test_testModels.py`), which judged every model against the default, and the **parallel** path of
the suite, where the model runs in another interpreter and `exu.sys` in the parent knows nothing
about it. The tolerance now travels with the result through the worker's result marker. One
feature, three runners — and the third was found by a test failing, not by thinking about it.

**Nothing else moved**: all 139 previous results are identical, and the ten are new.

<a id="rg10-6-7"></a>
### RG10.6.7 — a tolerance that was hiding inside a result (2026-09-24, #2632)

One model in the tree multiplied its own solution to make it fit the tolerance:

```python
exu.sys['testResult'] *= 1e-7  #result is too sensitive to small (1e-15) disturbances, so
                               #different results for 32bits and linux
```

The sentence is true and the mechanism was wrong: it states a tolerance by changing the number the
test compares, so the reference solution in `runTestSuiteRefSol.py` was `2.6388120463802584e-05`
for a model whose result is 263.88, and the comment beside it had to say *"original but too
sensitive to disturbances: 263.88120463802767"*. The model was in **no** other list —
`SensitiveTests`, `UnresolvedOnLinux`, `TestExamplesToleranceFactors`, the AVX2 update all pass it
by — so this was the only expression of the intent.

**The arithmetic is exact, which is what made the change safe.** `|raw*1e-7 - 2.6388...e-05| <
5e-14` is `|raw - reference| < 5e-7`. So the model states `exu.sys['testTolerance'] = 5e-7`, the
sentence that was in the comment stays as the reason, and the reference is the raw value —
**measured** on this machine, 263.88120463802585, which agrees with the old scaled reference to
its last digit. The test is neither stricter nor looser than it was yesterday.

This is the only model whose reference solution moved in the whole of RG10.6, and the old value
stands beside the new one in `runTestSuiteRefSol.py`, as the other entries carry their history.

<a id="rg10-6-8"></a>
### RG10.6.8 — the performance models, and the end of ExudynTestStructure (2026-09-24, #2632)

`AddTiming` read exactly one thing from the object it was handed — `timings`, through
`getattr— ` so the channel moved and the class went. `AddTiming(name, mbs, result)` appends to
`exu.sys['testTimings']`, which `runPerformanceTests.py` puts there before each model, and the
seven models begin with the same line as every test model.

Two things came out with the boilerplate, and both are the patterns this step exists to remove:

- **the last `testTolFact`.** `perfRigidPendulum.py` set `exudynTestGlobals.testTolFact = 1e5`
  against a runner tolerance of `1e-10`; it says `exu.sys['testTolerance'] = 1e-5` now, which is
  the same number without the multiplication;
- **the last reference solution inside a model**, `generalContactSpheresPerf.py`'s
  `testError = uSum - (-1.779402864432934)`, a copy of what
  `PerformanceTestsReferenceSolution()` holds.

`ExudynTestStructure` and `exudynTestGlobals` are **deleted**. Two users had to go first: the
worker bootstrap in `testRunnerTools.py`, whose fallback said *"until every model is converted"*,
and `runTestSuite.py`, which kept the instance as a **runner-internal accumulator** although the
value had come from `exu.sys` since RG10.6.1. Those are `TSScope` fields now, where the rest of
the runner's own state lives.

**Three things the run caught that reading had not.**

- `AddTiming` referenced `exu.sys` while `testRunnerTools.py` has **no module-level import of
  exudyn** — every other function in it imports exudyn inside the body, deliberately, because
  the module is also used by tools that never touch it. `AddTiming` does the same now.
- `perf3DRigidBodies.py` keeps its `useGraphics = False` default **below** the try/except block
  rather than above it, so the generic rename turned it into `(not testIsActive) = False`, which
  is not Python. Python said so at once.
- and that model's default was **False**, like `perfLargeMassSpringChain`'s: both default to
  `exu.sys.get('testIsActive', True)`, so that a standalone run behaves as it did.

All 13 single runs and all 7 performance tests pass, every test-suite result is identical, and
`exudynTestGlobals` appears nowhere in `python/` or `tools/` any more.

<a id="rg10-6-6"></a>
### RG10.6.6 — the recipe that never existed (2026-09-24, #2632)

This step was planned as *"update the documentation of the old pattern"*. The survey found that
**there is none**: not one hand-written page in `docs/dev/`, `docs/manual/`, `docs/howTo/`,
`CONTRIBUTING.md`, `CLAUDE.md` or `README.rst` ever named `exudynTestGlobals`, `modelUnitTests`,
`runUnitTests` or `TestInterface`. The nine lines of boilerplate propagated for seven years by
**copy-paste from the file next door**, and that is why every one of the 129 models had them and
why two of them had a version that differed.

So the step is not a correction but the recipe itself: `docs/dev/WORKFLOW.md` gains *What a test
model looks like* — the three lines, why the renderer calls keep their `if not testIsActive:`,
that the reference solution lives in `runTestSuiteRefSol.py` and not in the model, and the
instruction that RG10.6.7 exists to justify: **never scale a result to make it fit the
tolerance**, say `exu.sys['testTolerance']` instead. `CONTRIBUTING.md`, whose rule 3 is *"new
behaviour has a test"*, points at it.

Three counts were stale and are **measured**, not arithmetic'd: 139 test models (was 127), 171
examples (was 177, wrong before this work), and `pytest` collects 149 cases in 71 s, or 12 s with
`-n 8`. Two dead comments naming things that no longer exist are gone, one of which
(`#testInterface = TestInterface(...)`) was published on an example page.

<a id="rg10-6"></a>
### RG10.6 — closed (2026-09-24, #2632)

Eight sub-steps, seven commits, and the shape of the thing at the end:

| | before | after |
|---|---|---|
| lines a test model needs to say it is a test | **9 + an import** | **1** |
| reference solutions inside models | 98 + 1 performance | **0** |
| ways a runner learns a result | 2 (`exudynTestGlobals`, then `exu.sys`) | **1** |
| kinds of test file | 3 (unit test functions, models, mini examples) | **1** |
| tolerances hidden by scaling a result | 1 | **0** |

What it cost to be sure: the results of all 139 models were captured before the sweep and compared
after every batch. **One reference solution moved**, deliberately, in RG10.6.7. Everything else is
identical, in the serial suite, in the parallel suite, in `pytest` and in the performance runner.

Three defects fell out of the work that were nobody's plan: a window opened on the maintainer's
screen because the serial runner never called `SuppressAll`, the ten "unit tests" turned out not
to have been running since V1.6, and `itemHeaderEmitter.py` was writing files in the locale
encoding. None of them was found by reading.

<a id="rg3-10"></a>
### RG3.10 — one list, in one place (2026-09-24, #2599)

Both pages are written by `issueTracker.py` from the same store, and both listed every resolved
issue of every release: `CHANGELOG.md` **2440** lines, `docs/generated/trackerlog.md` **9153**, of
which about 2370 were a shorter rendering of what the other says in full — the tracker page adds
the author, the description, the remarks and both dates. Roughly 35 duplicated pages in the PDF.

The split the maintainer decided:

- **`CHANGELOG.md` is the current release**, with its release notes, and the table of every
  release stays above it — that table is the history at a glance and costs 16 lines. **124
  lines**, which is what somebody upgrading reads.
- **the tracker page is everything before it**, under a heading that says so:
  *"Resolved issues and resolved bugs before version 1.12"*, with a line pointing at the changelog
  for the current one. **8923 lines.**

Every issue is still published and still searchable; none is published twice.

**Two details decided where the text goes.** The pointer to the tracker page is in the **intro
prose, above** the release block, because `exudev release` cuts `RELEASE_NOTES.md` from the first
`## Version ` heading to the next one *or to the end of the file* — and with one release block
there is no next one, so anything below it would land in the release notes. And
`tools/issueTracker/trackerlog.html`, which keeps every issue for local browsing, needed nothing:
it is untracked and git-ignored already.

**The tests had to be turned around, not extended.** One of them asserted that the changelog's
current release comes *before* `## Version 1.10— ` which is now absent. It asserts the new
property instead: exactly one release section, the table still naming 1.10, and the ordering
within the release unchanged. A second test says the same from the other side, on the tracker
page. And a third covers the boundary the cut now always hits: a changelog with a **single**
`## Version` block, where `WriteReleaseNotes` runs to the end of the file.

<a id="rg11-1"></a>
### RG11.1 — the results monitor, evaluated (2026-09-24, #2610)

The step asked whether `MonitorResults(...)` can run **beside** a simulation, and said that if it
cannot, it is redundant because *"`PlotSensor` already does that"*. The maintainer asked for an
evaluation and a recommendation, not for code. Here is what was measured.

**How it blocks.** `ResultsMonitor.Run` (`resultsMonitor.py:693-721`) is **not**
`plt.show(block=True)`; it is an explicit loop, `while plt.fignum_exists(...)`, driven by
`plt.pause(updatePeriod)`, which also pumps the tkinter control panel through
`_ControlPanel.ProcessEvents` instead of a `mainloop`. So the monitor already owns a cooperative
event loop and gives it up only when the window closes. Two escape hatches exist: `once=True`
returns right after the first draw, and a suppressed UI (`Agg`, or
`exu.special.userInterface`) forces `once`, which is why the monitor is harmless in the test
suite.

**A premise of the step is wrong, and that matters for the conclusion.** `PlotSensor`
(`plot.py:172`) reads each file **once** and draws it; it has no re-read, no offset tracking and
no update loop. The incremental reader exists only in `resultsMonitor._IncrementalData`
(`:278-338`). So the in-script call is **not** a second way of doing what `PlotSensor` does —
it is the only way to watch a file grow, and dropping it would remove a capability rather than a
duplicate.

**What the package has to build on.** There is **no Python thread anywhere in `exudyn`** —
`threading` appears twice, in comments. The renderer's second thread is C++
(`GlfwClient.cpp:1591`, `std::thread`), and `multiprocessing` is used only for
parameter-evaluation pools in `processing.py` and `FEM.py`. Whatever runs beside a simulation
would be the first of its kind in the package.

**The four candidates, judged.**

- **a second thread** — cheapest to write and the worst fit: matplotlib is not thread safe, the
  monitor drives its own `plt.pause` loop, and the solver holds the GIL for long stretches inside
  C++, so the plot would freeze exactly while the simulation is interesting;
- **the renderer's own loop** — there is a periodic callback there, but it ties the monitor to
  a running renderer and puts matplotlib inside the GLFW thread's cadence. It also fails for the
  common case: a long solve with no renderer;
- **drop the in-script call** — ruled out by the measurement above: it is not redundant;
- **a second process** — **recommended.** The file is already the protocol; the monitor is
  already a command line tool (`python -m exudyn monitor`); nothing is shared, so no backend,
  GIL or thread-safety question arises; and the child dies with the script if it is started with
  the parent's lifetime in mind.

**The recommendation, in the shape it would take**, is a handful of lines in
`exudyn.misc.resultsMonitor`: a function that starts
`subprocess.Popen([sys.executable, '-m', 'exudyn', 'monitor', fileName, ...])` and returns the
handle, with a note that the file has to exist or the monitor waits for it — `WaitForData`
(`:392`) already does that. The in-script `MonitorResults` stays exactly as it is, for the case
where blocking is what the user wants.

Not built here, because the maintainer asked for the evaluation alone. It is proposed as
**RG11.3**.

<a id="rg6-2-18"></a>
### RG6.2.18 — the dialogs, from a shell (2026-09-24, #2624)

The step asked for the settings dialog on `simulationSettings` and left the question of **how to
open it** unanswered; the maintainer answered it on 2026-09-24: not from the renderer, where
changing a solver setting mid-step is not the harmless thing that changing a colour is, but from
model code or a shell — **`python -m exudyn dialogs xxx`**.

```
python -m exudyn dialogs vis      #the visualization settings
python -m exudyn dialogs sim      #the simulation settings
python -m exudyn dialogs help     #the keyboard and mouse commands of the renderer
```

Everything below the widgets was ready since RG6.2: `GetDictionaryWithTypeInfo()` is bound for
`SimulationSettings` too, `SettingsPrefix` writes the right name into the code line, and
`DefaultSettingsDictionary` falls back to the constructor for a structure that belongs to no
`SystemContainer`. So the step is a **command**, 40 lines in `python/exudyn/__main__.py`, added
to the `CommandTable()` that already held `monitor`, `plot`, `info` and `demo`.

Three decisions worth keeping:

- **it builds its own structure**, `exu.VisualizationSettings()` or `exu.SimulationSettings()`,
  and never a `SystemContainer— ` creating one attaches it to the render engine (#2625), and
  there is no model here anyway. Since RG6.2.20 a plain structure carries the values a user
  really starts from, which is what makes this honest;
- **it prints what the browsing was for.** The dialog writes into the structure, so when it
  closes, the command prints `ChangedSettingsCode(settings)` from RG12.3: the lines that set what
  you changed, ready to paste. Browsing a tree of 470 settings is only useful if you can take
  something away from it;
- **the command dialog is not among them**, as the maintainer said: a window that executes
  Python in the scope of a *running* model means nothing without a running model.

**And it opens no window in an automated run.** The command asks
`UIWindowSuppressed('Dialogs', ...)` first and returns 0 quietly — which is what makes it
testable at all, and what CLAUDE.md rule 11 is about. `test_commandLine.py` calls every form of
it, and the five commands of the table are checked against the list the usage prints.

<a id="rg6-2-24"></a>
### RG6.2.24 — a dialog without a renderer must look like one with it (2026-09-24, #2634)

`python -m exudyn dialogs` was one day old when the maintainer reported that its windows are
*"larger, a different font, and seem to be blurred"*. Two causes, and they compound:

- **the process was not DPI aware.** GLFW sets that when it creates the render window, so a
  dialog opened with **V** inherits it and is drawn at the real resolution of the display. Started
  from a shell there is no GLFW, so Windows draws the window at 96 dpi and **stretches the
  bitmap**: soft, and on a 175% display 1.75 times too large;
- **and the scaling it computed was wrong in the other direction.**
  `GetExudynDisplayScaling()` reads `displayScaling` out of the renderer's state and returned
  **1** when there is no renderer, so the content was laid out for an unscaled display and then
  stretched.

Both are fixed where they belong. `MakeProcessDpiAware()` is called once, in
`GetTkRootAndNewWindow`, **before the first window** — afterwards Windows refuses, and that
refusal is not an error here, it means something else has already done it. And
`GetExudynDisplayScaling(root)` asks tkinter when it cannot ask a renderer:
`root.winfo_fpixels('1i') / 96`, which is the display's true scaling once the process is aware of
it. Measured on this machine: **1.749** where it used to say 1.

So the dialog is now laid out at the same size the renderer's dialog is laid out at, and drawn
sharp instead of stretched to it.

**A test that had to be taught about its neighbours**: it asserts that without a renderer the
answer is 1, and another test in the same file registers a container in `exudyn.sys— ` so it
passed alone and failed in the file. It removes that key and puts it back.

<a id="rg6-2-25"></a>
### RG6.2.25 - an enum in a combo box, without the type in front of every entry (2026-09-24, #2635)

The maintainer, on `contour.outputVariable`: the entries of the list all begin with
`OutputVariableType.`, *"thus making it impossible to see what value it really is"*. Measured:
the longest of its 33 entries is 43 characters of which **19 are the prefix**, the combo box
is placed over the value cell and is therefore as wide as the value column, and the type name
it spends that width on is **already in the type column beside it**. The item types and the
solver types of the simulation settings are the same list with a shorter prefix.

What changed is only what the box **shows**. `EnumDisplayName(valueStr, vType)` takes
`vType + '.'` off the front, `EnumFullName` puts it back, and they sit in
`settingsUtilities.py` next to the other conversions - tkinter-free, so a test can reach them
without a window. The combo box shortens its list and its current entry in `StartCellEdit`,
and `OnCellCommit` lengthens the pick again, so **the tree cell, the settings structure,
`CheckType`, `ConvertString2Value` and the line `ChangedSettingsCode` writes all keep the full
name** - the dialog stays the only place the short form exists.

The round trip is required for **every enum the module has**, not for a hand-picked one: the
test walks `GetComboBoxListsDict(exudyn)` and asserts
`EnumFullName(EnumDisplayName(s, t), t) == s` for each of its values. The same box also edits
the bools, whose `True` and `False` carry no prefix and must come through untouched, which is
why `EnumFullName` leaves them, an empty string and an already complete name alone rather than
prefixing whatever it is handed.

<a id="rg10-2-1"></a>
### RG10.2.1 - the web view finds an author, and reaches the first issue (2026-09-24, #2636)

Two faults of `exudev issue serve`, both reported by the maintainer while using it.

**The search read four fields** - title, description, `workingRemarks`, `releaseNotes` - so
the two fields that say **who** did anything were not searchable, although the page shows an
author box and every issue carries `author` and `resolvedAuthor`. It reads every field of the
issue now, which also makes the file, the plan step, the version an issue was resolved in and
the dates searchable, at no cost worth measuring: one pass over 2,637 dictionaries. The
**number** stays a separate exact test, so `#2600` and `2600` both find that issue; the
existing test already records why a number search cannot demand exactly one hit - issues
reference each other in their text. Measured: `Claude-JG` finds **307** issues, and it is
case-insensitive like every other search.

**And the list sent the newest 400 rows.** The list is sorted newest first and has no paging,
so the cap did not shorten the list, it **deleted its older half**: nothing older than about
#2240 could be reached by any amount of scrolling. Measured before removing it: all 2,637 rows
are **470 KB** of JSON, built in **0.06 s**, over a loopback socket. `listLimit = 0` now means
no cap, and the page already said "N matching, M shown" when the two differ, so it needed no
change beyond the placeholder of the search box, which now says *search any field*.

<a id="rg3-10-1"></a>
### RG3.10.1 - one entry, printed by one function (2026-09-24, #2637)

RG3.10 stopped the two pages from holding the same issues. It did not stop them from printing
an issue in two different shapes, which is what the maintainer asked about next: *"the
Changelog and the issue tracker in the docs have a different format for the list of versions
and issues. Why?"*

There was no reason. The two renderers were written three months apart, for different
purposes, and each grew its own line:

```
changelog     - **1.12.50** `FIX` the dialogs are larger and blurred (#2634) - raised by X
tracker page  - Version 1.12.49: resolved Issue 2633: the dialogs are larger (fix)
                - issue author: X
                - description: ...
                - effort: LOW (within 2 hours)
                - date resolved: **2026-09-24 11:44**, date raised: 2026-09-24
```

So the changelog had no dates at all and the tracker page had no type badge, and the number
was spelled `(#2634)` on one page and `Issue 2633` on the other.

**`IssueEntry(issue, version, details=True)` is now the only thing that prints an issue**, and
both pages call it. The headline follows the maintainer's instruction exactly: the type as the
changelog wrote it, the **priority** and the **effort** in the same style right after it,
then `raised by:` and `resolved by:`, then the title and the number. The sub-list is the
tracker's, unchanged. Two decisions inside it:

- the **effort** badge carries its word - `LOW EFF` - and the priority does not. That is not a
  new rule: `LOW` and `HIGH` are values of **both** fields, and RG10.2 (#2600) had already met
  the problem in the web view, where two bare badges in one row could not be told apart;
- an **open** issue keeps the colour of its priority, which the page has always had and which
  is the only thing a reader of 270 open issues sorts them by. It is the badge that carries it
  now instead of a separate coloured prefix.

What the badges mean is said **once** per page, by `BadgeLegend()`, instead of `effort: LOW
(within 2 hours)` under each of 2,366 entries.

**The wrong sentence** the maintainer also reported: the changelog called the other page *"the
full issue tracker"* and told the reader that issues closed without being resolved are in
neither list. The page is not the full tracker - it holds the issues resolved **before** the
current release, plus the open issues and the known bugs - and the sentence about closed
issues belongs with the sentence about version numbers, which is where it is now.

**A test caught its own premise.** `testTheTrackerPageHoldsEverythingBeforeTheCurrentRelease`
asserted `'- Version 1.12.' not in text` over the whole page - and failed, because the
description of #2637 **quotes the old format**. It compares lines that start an entry now,
which is what it meant.

<a id="rg10-7"></a>
### RG10.7 — the plan holds the open work again (2026-09-24, #2638)

The plan says in its own header what a finished step keeps: *"a done step keeps one line here
- status, date, outcome, link to the log; an open step keeps its full text"*. It was not
following it. Measured before the cleanup: **971 of 1391 lines** were steps that are done,
and the longest of them - RG10.6 at 129 lines, RG6.2 at 67, RG9.2 at 41 - carried the problem
as it was first stated, the options that were weighed and how the work went. The maintainer
asked for it on 2026-09-24, naming RG3 and RG6.

Two rules did it, and both are about **where a fact already lives**:

- a step that ended in *"The original text follows"* is cut there, with the sentence. That
  text is the issue as it was raised, and `tools/issueTracker/issues/` has it, searchable by
  every field since RG10.2.1;
- the rest were rewritten to the outcome and the link. Nothing was dropped that is not in the
  log: **every one of the 52 done steps has a log entry**, which was checked before a line
was removed.

**1391 lines to 939.** No anchor, no step number and no group heading was lost - the check
that says so compares the sets before and after, and it caught a first attempt that had
deleted seven group headings, because a step block ran to the next anchor and a heading with
its introduction stands between two steps.

One piece of analysis lived **only** in the plan and is moved here rather than summarised:
the review of `GUI.py` of 2026-09-22, which is what the sub-steps RG6.2.1 to RG6.2.10 were
cut from. It follows as its own entry.

<a id="rg6-2-review"></a>
### RG6.2 — the review of `GUI.py` (2026-09-22, #2591)

*Moved out of the plan on 2026-09-24 by RG10.7 (#2638), unchanged. It is the reading of the
module that the sub-steps were cut from.*

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

<a id="rg6-2-26"></a>
### RG6.2.26 — a tooltip of a topmost dialog is topmost (2026-09-24, #2639)

The maintainer, on the dialogs of the renderer and of `python -m exudyn dialogs` alike: the
pop-up hints do not appear, and *"when I turn off alwaysTopmost and restart the dialog from
the renderer, it works"*. That second sentence is the diagnosis: a `Toplevel` without the
topmost flag can never be drawn above a window that has it, because on Windows topmost is a
**stacking band** and not an ordering within one. The tooltip was not failing to appear - it
was appearing behind the dialog.

**This is the third time the same mechanism has cost a defect in this dialog**: #2621, the
window of the changes that looked like a button doing nothing, and #2614 before it, where
the same window came up behind the dialog the second time it was opened. It is worth
stating once, here: *every* window the settings dialog opens has to carry `-topmost`,
or the dialog has to give it up while that window is open, which is what the changes window
does. There is no third option.

So `Tooltip.Place` sets `-topmost` on the window when it builds it, and lifts it each time it
is placed - the flag chooses the band, `lift` orders within it. The question the maintainer
raised alongside it, whether `alwaysTopmost` should default to **off** and the hints be
disabled with it, does not have to be answered: the dialog keeps the topmost it needs
in order to sit over the render window it blocks, and the hints work with it.

**Not tested, and why**: a tooltip is a window, and the suite opens none - the finding of
RG6.2 that the layer below the widgets is what a test can reach. Reading the flag back would
need a mapped window on the maintainer's screen, which CLAUDE.md rule 11 forbids and which a
headless CI could not do either. The evidence is the maintainer's own experiment (topmost
off, hints work) and the two earlier defects with the same cause.

**Confirmed by the maintainer, 2026-09-24**: *"the hints now appear with alwaysTopmost = True
AND with alwaysTopmost = False, so both works now, also in both modes"* - the renderer and
the command line.

<a id="rg6-2-25-1"></a>
### RG6.2.25.1 — the short name IS the value string (2026-09-24, #2640)

RG6.2.25 shortened the entries of the combo box, and the maintainer tried it: *"The list is
now good, but finally, it still writes OutputVariableType.Torque or so into the field as soon
as the combo box is collapsed."* The same problem one step later, and for the same reason -
the value column is as wide as the box was.

The first fix had kept the full name as the truth and shortened only what the list showed.
That was the wrong place to draw the line, and the report is what showed it: the **cell** is
also a place where the value is shown, and so are the window of the changes and the marking
of a changed row. So the rule is turned around: **the short name is the value string**, and
the full one exists in exactly one function.

It is one line of behaviour in `ConvertValue2String`, which is the single place where a value
becomes the string everything else compares and copies. Four places had to agree with it:

- `ConvertString2Value` and `CheckType` accept **both** spellings, because a value can still
  arrive written out - from a user typing it, or from an older script;
- `CheckType` lists the **short** names when it refuses one, since those are what the dialog
  offers;
- `ValueLiteral` puts the type back: `exu.OutputVariableType.Displacement` is what Python
  needs, and generated code is the only place that does;
- `OnCellCommit` stops expanding what the box gave it.

**Why the comparison did not break**: `ChangedSettings` compares the string the dialog shows
on both sides, and both sides come from `ConvertValue2String` - so they moved together. The
test that would have caught it if they had not is the one that asserts a single changed enum
is a single change, and not 470 of them.

Measured on the structures themselves: `visualizationSettings` has **two** enum settings and
`simulationSettings` **two** (`linearSolverType` and the `dynamicSolverType` that #2597 found
being edited as free text), and all four now read as `Displacement`, `_None`, `EXUdense`,
`DOPRI5` while their code lines carry the type.

<a id="rg10-2-2"></a>
### RG10.2.2 — the number is a field too (2026-09-24, #2641)

The maintainer typed `249` with status=all and got a result that looks arbitrary until one
knows the code: #2377, #1988 and #1142, which say 249 somewhere in their text, and **#244 and
#694, which say it nowhere a reader can see** - their `resolvedInVersion` is `0.1.249` and
`1.0.249`. And not #2497, which is what the three digits were typed for.

Both halves are one cause. RG10.2.1 made every field searchable **except** the number, which
kept the exact test it had always had, so a numeric search answered a question nobody asks:
*"is there an issue with exactly this number, or an issue whose text mentions it?"* The number
is now a substring like everything else, and `249` finds **#249 and #2490 to #2499**.

The two version hits stay, and they are not a defect: searching for `1.11.240` is a good reason
to have made the field searchable. They only looked arbitrary because the hit one expected was
missing.

<a id="rg10-7-1"></a>
### RG10.7.1 — what now (2026-09-24, #2642)

RG10.7 made the plan hold the open work again. The question it still did not answer is the one
the plan is opened for, and the maintainer asked for the answer to be written down: *what now*.

The section has three parts, which is one more than was asked for and is the shape the material
took:

- **Still open** - the 28 open steps, one line each, with the issue number where the step has
  one and a title short enough to scan. Nine of them are the plugin ABI, so they are one row;
- **Raised by the current work, and not yet a step** - the GitLab `regenerated_files` failure
  that cannot be reproduced here, the documentation of the finished steps, and four things that
  earlier steps deliberately left open (#2608 remember the window, #2616 a hook for
  `forceQuitSimulation`, #2423, #2541, #2497). **None of them is planned**: they are written
  where they were found, and the section only makes them visible in one place;
- **Recommended next** - five items in an order, each with the reason, because an order without
  one is an opinion. The first is the 1.13 release, and the argument for it is not that it is
  most important: it is the only item on the page that needs **other people's time**.

What the section must not become is a second backlog. It carries a number, an issue and a short
title, and the step keeps its full text in its group - the same rule RG10.7 applied to the
finished steps. The open issues that are not steps are not listed at all: there are hundreds of
them and they belong to `exudev issue serve`, which since RG10.2.1 and RG10.2.2 can actually
find one.

<a id="rg6-6"></a>
### RG6.6 — only the outermost idle operation pumps (2026-09-24, #2643)

The first test of Exudyn **with graphics on macOS** (maintainer, 2026-09-24): *"Basically
works. ... after some steps crashed."* `SIGABRT`, a fatal Python error in
`PyEval_RestoreThread`, macOS 14.5 on arm64, Python 3.13, after interacting with the
visualization dialog.

The crash report has **103 frames of the crashed thread**, and they say the whole thing. Read
from the bottom, the boundary between Python, Tcl and C++ is crossed **four** times:

```
tk.mainloop()                          the interactive dialog of interactive.py
  after() timer -> PythonCmd
    SC.renderer.DoIdleTasks(0)         <- idle operation 1
      PyProcessExecuteQueue()          the queued Python of the V key:
        pybind11::exec -> the settings dialog
          wait_window -> Tk_TkwaitObjCmd
            a Tk binding -> PythonCmd
              UpdateSettingsStructure
                SC.renderer.DoIdleTasks(0)   <- idle operation 2, and the defect
                  glfwPollEvents()
                    NSApplication nextEventMatchingEventMask
                      the Cocoa run loop redraws the TKContentView
                        Tcl_DoOneEvent -> PythonCmd
                          PyEval_RestoreThread -> fatal error -> abort()
```

**Why macOS and not Windows.** GLFW cannot render from a second thread on macOS, so the
renderer is always single-threaded there: the event pump lives inside `DoIdleTasks()` instead
of in a thread of its own. And `glfwPollEvents()` on macOS runs the **same** Cocoa run loop
that tkinter draws on - so polling events redraws the dialog and calls back into Python, at a
point where `_tkinter` has already handed the GIL to Tcl. On Windows polling GLFW events does
not touch the Tk event loop, and the default renderer is multithreaded, so neither half of
the mechanism exists.

**The fix is a depth, not a platform test.** `GlfwRenderer::idleOperationDepth` counts the
idle operations on the stack, held by a `ScopedIdleOperation` so that it comes back down when
the queued Python throws. Two places ask it, and both are in the **single-threaded** path:
`VisualizationSystemContainer::DoSingleIdleOperation` runs `PyProcessExecuteQueue()` only at
depth 1, and `GlfwRenderer::DoRendererTasks` polls and runs the queue only at depth 1. A
nested operation still **renders**, which is what the dialog wanted from it: the live update
of `dialogs.multiThreadedDialogs` keeps working.

**What this does to the other platforms: nothing, by construction.** Both guarded blocks sit
inside `if (!useMultiThreadedRendering)`, and Windows and Linux render multithreaded by
default - the block does not execute there at all. A user who turns multithreaded rendering
off gets the guard, and the same re-entrancy is latent for them, so that is a fix rather than
a change. This is what the maintainer asked for: *"Keep the other platforms unchanged, only
adapt for MacOS. The fix ... under the assumption that it is single-threaded."*

**Not verified by me**: there is no macOS here, and the Windows suite cannot reach the defect
- 126 models, 23 mini examples and the pytest suite pass, which shows only that nothing else
moved. The second event pump also had a second, platform-independent fault worth naming: it
would have started the **next** queued Python while the previous one was still on the stack.

The workaround, if the guard is not enough: `dialogs.multiThreadedDialogs = False`, which
removes the nested call entirely. Its own description has said *"may cause problems on some
platforms"* since long before this.

<a id="rg10-8"></a>
### RG10.8 — exudev on three platforms (2026-09-24, #2644)

*"regarding exudev: this tool is also needed for linux and MacOS"* (maintainer, 2026-09-24).

The first thing was to find out how much was actually wrong, and the driver makes that easy: it
**plans** its steps before it runs them, so `--dry-run` prints the argv of every step without
touching anything. Every command was planned under WSL with `--dry-run --no-conda`, and the
answer is that most of it was portable already - the conda lookup tries `bin/conda` before
`Scripts\conda.exe`, the wheel lookup carries no platform tag, and the stale package copy of
#2560 is found through `build/lib.*`, which is the glob and not the Windows name. `generate`,
`build`, `test`, `docs`, `perf`, `examples`, `issue` and `env` plan the same steps there as here.

**Three places were Windows-only**, and each is wrong in its own way elsewhere:

- **`clean`** matched `build/lib.win-amd64-*`, `build/temp.win-amd64-*`, `build/bdist.win32` and
  `build/bdist.win-amd64`. setuptools names those directories after the platform that built
  them, so on linux and macOS the command removed **nothing** and said so cheerfully. The
  patterns are built from `sys.platform` now. On linux the native directories are then found
  twice - by the platform glob and by `--linux` - so the target lists are deduplicated;
- **`docs --open`** called `xdg-open`, which **macOS does not have**. One opener per platform:
  `cmd /c start`, `open`, `xdg-open`;
- **`linux`** wrapped the manylinux docker command in `wsl -e bash -lc`. On linux there is no WSL
  to go through - the command itself is identical, so `InWslIfNeeded()` decides the wrapper and
  `LinuxBuildRoot()` decides whether the path has to be translated. On **macOS it is refused**
  with the reason: the image is x86_64 and an ARM Mac would emulate it, which is not what a
  release wheel should be built with.

**And it is tested, which needs no second machine.** Because the driver plans before it runs, a
test can pretend to be any platform and read the steps back: `python/testing/test_exudev.py`
sets `sys.platform` and `runner.onWindows`, then asserts the clean patterns, the opener and the
argv of the container - nine cases, from whichever platform happens to run pytest.

<a id="rg6-1"></a>
### RG6.1 — OpenVR is removed (2026-09-24, #2645)

The step the rendering revision was waiting for, planned as revision2026 step R11.1 and moved
here when that plan closed. What OpenVR cost was never the feature - it is compiled only
behind `__EXUDYN_USE_OPENVR`, which **no shipped wheel has ever set**, so nobody who installed
Exudyn could use it. It cost the guards inside the renderer, a public settings structure
documenting something unreachable, an entry in the render state, a source file in every build
list, and **2 MB of vendored SDK and prebuilt binary** in a repository whose rule is to stay
small.

**What went, in one list:**

| where | what |
|---|---|
| `src/Graphics/` | `OpenVRinterface.cpp` and `.h`, 40 KB |
| `GlfwClient.cpp` | six `#ifdef __EXUDYN_USE_OPENVR` blocks: the extern, the instance, init, render, state, shutdown |
| `GlfwClient.h` | the `#define` comment and `SetProjectionMatrix()`, whose only caller was OpenVRinterface |
| `VisualizationSystemContainerBase.h` | the classes `OpenVRaction` and `OpenVRState`, and the `openVRstate` member of the render state |
| `MainSystemContainer.cpp` | the `openVR` entry of the render state dictionary |
| `definitions/` | `VSettingsOpenVR` and `interactive.openVR` - four settings |
| `setup.py`, `pyproject.toml` | `useOpenVR`, `--openvr`, the macro, `openvr_api.lib`, `-lopenvr_api`, and the copy of the DLL into the package |
| the build lists | `sources.json`, `msvc/cppsrc.vcxproj` and its `.filters` |
| the repository | `include/openVR/` (1.2 MB), `libs/libs64/openvr_api.dll` (808 KB) and `.lib` |
| Python | the example `openVRengine.py` and its action manifest |
| documentation | the *OpenVR* section of *Advanced topics*, the `openVRstate` sentence in `GUI.md`, the Valve licence block |

**What deliberately stayed.** Three things that look like OpenVR and are not:

- the **`else` branch of `SetProjection`**, which was commented `//openVR`. It runs whenever
  the render state carries a projection matrix that is not the identity - and `SetState`
  accepts `projectionMatrix` from Python, so it is a general feature that OpenVR happened to
  be the only user of. The comment says what it really is now;
- **`projectionMatrix` in the render state**, for the same reason;
- the **acknowledgement** of Aaron Bacher in `gettingStarted.md`, who helped integrate it. The
  feature goes; the fact that somebody did the work does not.

**The reference that had to move**: `parameterConversionTestReference.txt` records the outcome
of writing a probe value into every parameter through every access path, so it carried three
rows of `VisualizationSettings.interactive.openVR`. It was re-recorded with
`recordReference = True`, which the model provides for exactly this, and **only those three
rows differ**.

Users are told rather than left to find out, which is what the step asked for: the release
note of #2645 says it, and `docs/manual/revisions.md` has a paragraph under *What can break a
script* saying that a script which needs OpenVR stays on Exudyn 1.11.

<a id="rg3-12-1"></a>
### RG3.12.1 — install first, then run something (2026-09-24, #2646)

The first of the six sub-steps of RG3.12, and the smallest: *"Installation instructions are
after Run a simple example in Python. Should be switched."*

The cause is a **toctree at the end of a page**. `gettingStarted.md` lists its sub-pages -
the installation instructions and the FAQ - in a toctree after its last section, and a toctree
places the pages it lists at the position it stands in. The last section of that page is *Run a
simple example in Python*, so the sub-pages came after it. Moving the toctree up would have
nested the installation under *Further notes*, which is where it would then sit in the sidebar.

So the **example became a page of its own**, `gettingStartedExample.md`, and the toctree lists
the three in the order a reader needs them: install, run something, then the questions. Its
first sentence used to say *"After performing the steps of the previous section"*, which was
wrong in the old order and is right in the new one - it now names the installation.

**And three LaTeX relicts**: `-{}-pre`, `-{}-pre` and `-{}-version` in
`gettingStartedInstall.md`, which is how LaTeX writes a double dash that must not become an en
dash. The conversion of revision2026 step R7.1.5 has a rule for it (`autoGenerateHelper.py`
maps `-{}-` to `--`) and these three were converted before that rule existed. A reader who
copied the line got `pip install exudyn -{}-pre`, which pip refuses. No other hand-written page
has one.

<a id="rg3-12-2"></a>
<a id="rg3-12-3"></a>
<a id="rg3-12-4"></a>
### RG3.12.2 to RG3.12.4 — three pages where there were three copies (2026-09-24, #2646)

One entry for three sub-steps, because they are one piece of work: the new pages cross-reference
each other, so a commit that adds only one of them fails the strict documentation build.

**RG3.12.2, `docs/dev/BUILD.md`.** The build was described in three places that disagreed. The
user manual had *Build and install under Windows / Mac OS X / Ubuntu* - about 170 lines that
still said *"go to `main` of your cloned github folder"* (the `main/` level went in revision2026
step R3.1), still described Ubuntu 18.04 with Python 3.6, and still offered
`python setup.py bdist_wheel`, which current setuptools does not have. `docs/howTo/
buildFromSource.md` was the accurate one and nobody found it. `docs/dev/README.md` had a third,
short version.

The new page is the how-to note **plus** what was only in the manual (the macOS section, the
RaspberryPi, the WSLg software-OpenGL flag), reorganised the way the maintainer asked: a
**platform-independent part first** - what you need everywhere, and the two commands that are
the same on every platform - and then Windows, Linux, macOS. `docs/howTo/buildFromSource.md` is
deleted; the user manual keeps **one** section, *Build Exudyn from source*, which says when a
user needs it at all and links here. `gettingStartedInstall.md` went from **317 to 153 lines**,
and the stale *How to install Exudyn and use the C++ source code (advanced)* section - Visual
Studio 2017 and a `main_sln.sln` that has not existed for years - went with it.

**RG3.12.3, `docs/dev/GETTING_STARTED.md`.** *"It starts with Environments, it starts already
with the existing environment - but where does it come from?"* Now it comes from step 3 of seven:
install git, a Python and a compiler; **get the code**; create the environment; set up the clone;
build once; run the tests once; where to go next. It is the first page in the documentation that
contains a `git clone` at all - over **https or ssh**, with the sentence that says which to
choose and that https asking for a password wants a token. `WORKFLOW.md` §0a, the one-time setup
per clone, moved here, where a reader meets it before the versioning rules rather than after.

**RG3.12.4, `docs/dev/GIT.md`.** The branches, the everyday loop (`status`, `pull`, `add`,
`commit`, `push`), what a commit message looks like and why, branching for a larger change,
what a contribution from outside has to provide, and a table of *things that go wrong and the
way back*. Written for the **command line**, because that is what a VS Code user has in front of
them. It is the mechanics only: what a change must pass stays in `WORKFLOW.md`.

And `docs/dev/README.md` stops being the third build description: its *Building and running*
section is a table pointing at the three pages, plus the paragraph on editors that RG3.12.6 will
extend.

<a id="rg3-12-5"></a>
### RG3.12.5 — WORKFLOW.md in the order the work is done (2026-09-24, #2646)

Its ten sections were numbered **0, 1, 2, 0a, 2a, 2b, 3, 4, 5, 6**: the letters are what a
section gets when it is added after the numbering is fixed, and the result was that the one-time
setup of a clone (§0a) stood **after** versioning, and the branches and the CI (§2a, §2b) between
versioning and the commit tiers.

They are **1 to 10** now, in the order the work is actually done: environments, the issue
tracker, versioning, branches and remotes, testing, continuous integration, commit tiers, the
four gates, committing, the build and release scripts. Two changes beyond renumbering:

- **§0a is gone**, because it is step 4 of `GETTING_STARTED.md` now, where a reader meets it
  before the rules rather than after them. What is left here is a pointer;
- **testing is its own section.** *Which tests run when*, *what a test model looks like*,
  *pytest*, *running the suite in parallel*, *reproducible vs sensitive tests* and the platform
  differences were sub-sections of **continuous integration**, which is not where a developer
  looks for them: CI is where the tests also run, not what they are.

Every reference to a number that moved was rewritten - `§4` to `§8` for the gates in three
places and in `GIT.md`, `§2` to `§3` for the version files, `§2a` to `§4` in
`GETTING_STARTED.md`, `§1` to `§2` in `GIT.md` - and the references to the *plan* and to the
info document, which use the same `§` sign, were left alone.

<a id="rg3-12-6"></a>
### RG3.12.6 — which editor, said correctly (2026-09-24, #2646)

The maintainer: *"It says that Microsoft Visual Studio 2022 is the main development platform:
this is only partly true. I use it for debugging, but the Claude Code integration motivates me to
work much more in VS Code, also for the Python side, working with definition files, etc. Most
co-developers will work from VS Code."*

Four places said the old thing, and the invariant is the one that matters, because every
restructuring step is checked against it. What it says now is that **the capability is the
invariant, not which editor is primary**: stepping from a Python script into a C++ item with one
debugger is what must be preserved, and that is VS2022. The everyday work - the Python package,
the definition files, the documentation - happens in VS Code, and
`tools/setupLocalWorkspace.py` has been preparing both for some time: it writes the Visual Studio
solution **and** the `c_cpp_properties.json` without which VS Code cannot follow a C++ include.

Changed: the invariant in the shared info document §7 (with the date and who decided it), the two
places in `CLAUDE.md`, the invariant list of the developer README, and the sentence of the user
manual that tells a reader which editor to install - which now says that VS2022 is what compiles
and debugs the C++ on Windows, and that the development itself happens in VS Code.

<a id="rg3-8-1"></a>
### RG3.8.1 — one name, two formats (2026-09-24, #2594)

The maintainer drew the three missing SVGs of the contact-friction figures, so the pair that
RG3.8 asked for could be tried for the first time: **SVG for the browser, PDF for the PDF, one
name**. In `definitions/itemDefsObjects.py` the three `.. figure::` directives read
`docs/figures/<name>.*`, and Sphinx picks the candidate the builder supports - `image/svg+xml`
first for html, `application/pdf` first for LaTeX. Verified in the built html: all three
`<img>` elements point at `.svg`. The LaTeX side is **not** verified here, because the PDF is
built only for a release and is in no gate (RG3.3, decision D17); the `.pdf` files exist, which
is what that builder needs.

The widths stay as the old `.tex` had them, which is what the maintainer asked for: 600 for the
sketch, 600 for the sticking position, 700 for the normals.

**Which figures are worth an SVG next**, measured rather than guessed. Of the 40 figures the
documentation references, **17 are shown as `.png` although a vector original (`.pdf`, two of
them also `.eps`) is already in `docs/figures/`** - for those the SVG is a conversion, not a
drawing, and nothing else has to change but `.png` to `.*`:

> `CommonTangents3D`, `ConvexRolling`, `DrawSystemGraphExample`, `MarkerSuperElementRigid`,
> `ObjectFFRFsketch`, `ObjectJointRollingDiscSketch`, `RotationAxisAngle`,
> `RotationAxisAngleDerivation`, `SphereHollowsphereContact`, `SphereSphereContact`,
> `degrees_of_freedom`, `elementaryRotationX`, `elementaryRotationY`, `open_closed_loop`,
> `plotSpringDamper`, `spectralRadiusZeta0`, `triangleNormal`

The remaining 20 have **no** vector original, and there the question is what the image is. Counted
the distinct colours of each (a drawing has hundreds, a rendering or a screenshot has thousands):
**eight are drawings** and would have to be redrawn, **twelve are renderings, screenshots or
photographs** and stay raster, because an SVG of a screenshot is a PNG in an XML wrapper.

| drawings, no vector original | current size |
|---|---|
| `pendulum` | 598x629 |
| `pendulumConstraint` | 593x632 |
| `theoryRotationsHTchangeOfFrame` | 521x566 |
| `kinematicTreeCRBmass` | 1138x1075 |
| `kinematicTreeRNEA` | 1147x1090 |
| `theoryRotationsTaitBryanAngles` | 1010x958 |
| `RotationsSequences` | 1524x1288 |
| `TutorialBeams` | 2280x1550 |

The first three are where it pays most: they are **line drawings under 640 pixels wide**, which
is the case where a reader who zooms sees the pixels.

<a id="rg10-9"></a>
### RG10.9 — one capital letter, invisible on Windows (2026-09-24, #2647)

The `regenerated_files` job had been red since 1.12.48 and could not be reproduced here. The
maintainer supplied the GitLab log of 1.12.61, and it names the fault in one line:

```
ERROR: tools/generators/structureHeaderEmitter.py exited 0 but produced no output:
  | src/Autogenerated/pybind_modules.h  (does not exist)
```

The file tracked in git, and the one `src/Pymodules/Pybind_modules.cpp` includes, is
**`Pybind_modules.h`** with a capital P. The emitter wrote `pybind_modules.h`, and `generate.py`
declared the same lowercase name among that stage's outputs.

**On Windows those are one file**, so the emitter overwrote the right header and the post-check,
which uses `glob`, found it. **On linux they are two.** And the consequence there is worse than a
red job: `WriteTextIfDifferent` **refuses to create a file that does not exist** - it printed
*'illegal file'* and returned - so `Pybind_modules.h` was **never regenerated on linux at all**.
A stale header would have been used had anything changed in the settings structures.

Three changes, and the third is the one that matters for next time:

1. **one spelling**, `Pybind_modules.h`, in the emitter and in `generate.py`;
2. `WriteTextIfDifferent` **creates** a file it is asked to write, instead of refusing it. The
   guard was meant to catch a typo in a path; `generate.py` has checked every declared output
   since #2526, which is a better place for that;
3. a declared output is now checked **case-exactly** on every platform. `glob` is
   case-insensitive on Windows, which is precisely why this could not be seen here; a directory
   listing is exact everywhere. The message says which it is: *the file on disk is
   "Pybind_modules.h"*.

**Verified by reproducing the CI failure on Windows**: putting the lowercase name back into
`generate.py` makes `regenerate.py` print the same error the GitLab job printed, with the
spelling named. A comparison of all 37 declared single-file outputs against `git ls-files` with
exact case finds this one and **no other**.

<a id="rg3-13"></a>
### RG3.13 — what the thing is, not when it changed (2026-09-24, #2648)

The maintainer, on the header of a test model: *"the reader is interested about what the model
does, not how it was called before and when it was changed."* The rule went into `CLAUDE.md` 6a
and `CODING_STYLE.md` §6 with #2646; this step applied it to the **887 mentions of
`revision2026` in 257 files** outside `docs/revision/`.

**Tier 1, the models** - which are documentation pages. Ten of them opened with *"one of the ten
small tests that lived in `python/testing/modelUnitTests.py` from 2019 until revision2026b step
RG10.6.5 ..."*. Each now says what it computes, **read out of the model itself**: a cantilever of
`ANCFCable2D` elements solved once dynamically and twice statically, a slider-crank solved as an
index 3 **and** an index 2 system, constraints switched on and off through `activeConnector`, and
so on. The one sentence of history that stayed is a fact about the model - it compares against a
reference written into it, so its reference solution is 0.

**Tier 2, the definitions** - the reference manual. One **published** description named a plan
step (`dialogs.fontScaling`); `definitions/README.md` opened with what it replaced; 25 file
headers carried *"(revision2026 step R4.3)"*.

**Tier 3, the manual and the developer pages.** About fifty. Two references stayed, because they
are *about* the plan: `CODING_STYLE.md` stating the citation convention, and the developer index
linking to the revision logs. Where a sentence only made sense as a promise - *"step R5.3 wires
the LEST tests into the Debug configuration"*, *"step R1.5 adds a pre-push hook"* - it now says
what is true today.

**Tier 4, the comments of `src/`, `tools/` and `python/`.** 652 of the 887 were parentheticals or
appended clauses, and a rule could take them: a parenthetical carrying an issue keeps the issue,
one carrying only a step goes, and a prepositional phrase goes with its preposition.

**And this is where the automation stopped, after breaking something.** A reference is often
wrapped across two comment lines, which no line-based pattern sees, so a third pass allowed a
line break inside the phrase. One of its matches began on a **code** line and ended on a comment
line: it removed the newline between them and merged prose into code -
`examplesDir = pythonDir + 'Examples/' each of these holds one kind of file (#2513)`, which is a
syntax error and was caught by the next `regenerate`. Every file was then **rebuilt from its
committed content** with the line-based rules only, and the check that says it is sound is that
**every changed file has the same number of added and removed lines** - no newline was removed
anywhere. The lesson is small and general: a pattern that may cross a line break must not be let
near source code.

**235 mentions are left**, none of them in a published page, and each needs a sentence written
for it. That is RG3.13.1 (#2649).

<a id="rg3-8-2"></a>
### RG3.8.2 — the figures, and what a screenshot of an algorithm was hiding (2026-09-25, #2594)

The maintainer drew three more SVGs and answered the rest of the list from RG3.8.1.

**Three more are `.*` candidates**: `pendulum`, `pendulumConstraint`, `RotationsSequences` - the
three line drawings under 640 pixels that the measurement had put first. They have **no `.pdf`**,
so the LaTeX builder falls back to the `.png`, which therefore stays; the html uses the SVG.

**Two of them were never figures at all.** `kinematicTreeRNEA.png` and
`kinematicTreeCRBmass.png` are **screenshots of a typeset LaTeX algorithm**: the old
`itemDefinition.tex` had both as `algorithm` environments, the RST conversion could not carry
them, and somebody photographed the output. They are the algorithms again, written out of the
old `.tex` as two numbered lists with the math inline - Featherstone's recursive Newton-Euler and
the composite-rigid-body algorithm - so the equations are now text a reader can select, search
and zoom.

Writing them took two attempts, and the reason is worth keeping: the `\onlyRST{}` block of a
definition is converted to Markdown, and the converter **strips the leading indentation of a
continuation line**. An RST `#.` auto-numbered list came through as the literal characters `#.`,
and a nested list lost its nesting. So the algorithms are **flat numbered lists** whose loop
bodies are one item each - which reads well and cannot be broken by the converter.

**`intro2.jpg` is the title picture of the PDF again.** It was the title page of the old LaTeX
document, and nothing had replaced it: `pdfIndex.md` is the root of the PDF build and is excluded
from the html one, so an image there appears in the printed documentation and nowhere else.

**Eight files left the tree**: the three contact-friction `.png` (the `.svg` and the `.pdf` cover
both builders), the two kinematic-tree screenshots, `intro1.png`, which nothing referenced, and
the two `.eps`, which no builder can choose.

**Both builders were checked this time**, which RG3.8.1 could not claim: the html references
`.svg` for all six candidate figures, and a LaTeX build - `sphinx -b latex -t pdf`, which needs
no LaTeX installation to resolve images - copies `ContactFrictionCircleCable2D*.pdf`,
`pendulum.png`, `pendulumConstraint.png`, `RotationsSequences.png` and `intro2.jpg`. That is the
proof that deleting the three PNGs was safe and that keeping the other three was necessary.

**Found on the way, and not acted on**: seven `.png` that no page references -
`ObjectRigidBody`, `PrismaticJointX`, `RevoluteJointZ`, `RevoluteJointZ2`, `SphericalJoint`,
`TutorialRigidBody1`, `UniversalJoint` - and `ExudynLOGO1.7.jpg`, an older logo. The seven are
the same kind of loss as the four `.pdf` that RG3.8 is about, so they belong to that step and not
to a deletion.

<a id="rg3-8-2-toctree"></a>
**And the tutorials** (#2650): the toctree of `tutorial.md` stood at the end of the page, inside
its only section, so the four sub-tutorials appeared **below** *Mass-Spring-Damper tutorial*
instead of beside it - the same cause as the installation instructions of #2646. It is at the top
level now, and a check of every toctree in the hand-written pages says this was the last nested
one.

<a id="rg3-8-3"></a>
### RG3.8.3 — a reference form the audit did not know (2026-09-25, #2651)

RG3.8.2 reported seven unreferenced `.png` and left them for the maintainer, who answered that
**six of them are pictures of an item** and one is an old tutorial figure. He also found what the
audit had got wrong: `ObjectRigidBody` **is** shown, through
`\addExampleImage{ObjectRigidBody}` in the `classDescription` - a command that names the file
**without its extension**, which a search for `figures/<name>.png` cannot find.

So three of the six were already on their page and three were not. They are now:

| picture | item | why there |
|---|---|---|
| `RevoluteJointZ2` | `ObjectJointRevoluteZ` | the page showed one picture of a single link; this one shows the **two bodies** the joint connects, with the axes labelled |
| `SphericalJoint` | `ObjectJointSpherical` | the item, drawn |
| `UniversalJoint` | `ObjectJointGeneric` | there is no universal joint item: the generic joint **is** one when a single rotation axis is constrained, which its description already explains |

`TutorialRigidBody1.png` is the figure of a tutorial that no longer exists, and is gone.

**The audit is corrected**, and with it the count: of the 13 files in `docs/figures/` that no
page can reach, **ten are the vector originals** of PNGs that are in use - the group that RG3.8
wants to become `.svg` candidates - `ExudynLOGO1.7.jpg` is an older logo, and two are the figures
that are still lost, `generalContactANCF2Dcircle.pdf` and `generalContactSpheres.pdf`.

**A correction to RG3.8 while looking**: the third lost figure, `ObjectJointALEmoving2D.pdf`, is
not unreferenced at all - it sits in an `\ignoreRST{}` block, so it is in the **PDF and not in
the html**. That is a different fault from the other two and needs an `\onlyRST{}` twin rather
than a new figure.

**What this says about the method**: an audit is only as good as the reference forms it knows,
and this one knew two of three. A check in the gate would have said so on the day the third form
was introduced; that is worth proposing rather than doing in this step.

<a id="rg10-10"></a>
### RG10.10 — a live file that reads like a dead one (2026-09-25, #2653)

The maintainer read the header of `tools/generators/definitionLoader.py` and asked: *"is
definitionLoader still used and needed? Or if it is a code that was used to convert old
definitions - why not just say that this function is not used, but kept like as a backup?"*

**It is used.** Measured by following the imports:

| user | what it takes |
|---|---|
| `itemDocsEmitter.py` | `LoadItemDefinitions` - the item pages of the reference manual |
| `structureDocsEmitter.py` | `LoadStructureDefinitions`, through `structureModel.LegacyStructures()` - the settings pages |
| `structureModel.py` | `structureModules`, the **list of definition modules** - so the structure header, docs and stub emitters all depend on this file for that alone |

Remove it and three generators stop.

**Why it reads as dead** is the interesting part, and it is the fault `CLAUDE.md` 6a is about,
in a file header. The old text opened with *"hands them to the generators in the form their old
line parser produced"* - history before subject - and closed with two promises:
*"Removed once the generators read definitions/ directly"* and *"this adapter is deleted with
them"*. A promise in a header ages into a claim that the file is obsolete. The header now names
the three users, says what the conversion does, and states that the removal is work of its own
rather than something the file can announce.

**And one thing really was dead**: `itemModel.LegacyItems()` with its two tables
`legacyItemHeaderKeys` and `legacyLineDefinition`. Nothing has called it since the item emitters
stopped using the old form - the two documentation emitters build their own template. Removed,
and the regeneration is byte-identical.

<a id="rg6-2-27"></a>
### RG6.2.27 — the command window is in the model again (2026-09-25, #2654)

The maintainer: *"I press X in the renderer, and usually I have the context (like mbs, SC, ...)
and can edit something, print something ... Right now, it seems that the context is gone, but it
worked with earlier exudyn versions."*

It did, and the history says exactly why. Until RG6.2.1 (#2595) the command window was a Python
string inside `rendererPythonInterface.cpp`, and **every** such string was executed by

```cpp
py::object scope = py::module::import("__main__").attr("__dict__");
```

whose own comment reads *"use this to enable access to mbs and other variables of global scope"*.
The dialog code therefore ran **inside `__main__`**, so its `exec(commandString, globals(), ...)`
saw the model. RG6.2.1 moved the dialog into `exudyn.misc.GUI`, where `globals()` is the module -
and a module has no `mbs`. The C++ still executes its one-line wrapper in `__main__`, which is
why the window opens and only the commands fail.

`ModelScope()` returns `vars(__main__)` and the command runs in it. **One dictionary, not two**:
the old line passed `globals()` and `locals()`, so `k = 5000` landed in the locals of the button
handler and was gone when it returned - in a window whose label offers to *change* a running
model. Now an assignment stays in `__main__`, which is where the model is.

**Tested without opening a window**, which is the point of making the scope a function: three
tests in `test_guiValues.py` - the scope IS `vars(__main__)`, a command sees a variable of the
model and writes one back, and the module namespace is not the model namespace.

`tools/checkExtras.py` had to learn one thing for it: `__main__` is not in
`sys.stdlib_module_names`, because that lists the modules that come as files, so `import
__main__` looked like a package to install.

<a id="rg3-14-8"></a>
### RG3.14.8 — the rules for a description, in one place (2026-09-25, #2655)

`definitions/README.md` said how a member is written and nothing about the text inside it, so the
conventions of the reference manual's source lived only in the existing descriptions, and a
developer learned them by copying a neighbour. A new section, **"Writing a description"**, says
which five fields are descriptions and what each becomes, that the text is Markdown with LaTeX
mathematics and that nothing else is supported, gives one line per construct, and ends with the
three commands that check it.

The pointer to it, and nothing else, is repeated: the header of all **24** definition files that
carry a description names the section, and `CLAUDE.md` rule **6b** does the same for a Claude
session. This is the shape the rest of RG3.14 uses - each sub-step rewrites its own line of that
section and no rule is written twice.

Two facts found while writing it, and stated as they are:

- **A citation needs no macro.** `conf.py` appends a Markdown link definition for every key of
  `docs/bibliographyDoc.bib`, so `[ZwoelferGerstmayr2021]` written directly in the text already
  becomes a link into the generated references page. All 31 `\cite` calls in `definitions/` are
  single-key, so there is nothing the native form cannot say.
- **`latexToMarkdown.ReportUnknown` is called by nothing.** It would name a macro the converter
  does not recognise; it is dead code, so an unknown macro reaches the page silently today. That
  is what RG3.14.7 changes, and the new section says plainly that the page is the only place a
  mistake shows until then.

<a id="rg3-14-1"></a>
### RG3.14.1 — an abbreviation is ABRV:ODE2 (2026-09-25, #2655)

The 178 abbreviation calls of `definitions/` were written in **seven** LaTeX spellings - `\hac`,
`\hacs`, `\acf`, `\acl`, `\acs`, `\acp`, `\ac` - and `ConvertInline` rendered all seven
identically, as `` {ref}`KEY <KEY>` ``. Seven names for one macro. They are now one form with no
backslash and no braces:

```
ABRV:ODE2
```

The key ends where the word ends, and no delimiter is needed: measured over `definitions/`, **not
one** of the 178 was followed by an alphanumeric character. The LaTeX spellings stay in the
converter, because the hand-written chapters of `docs/manual/` still use them.

**`tools/checkDefinitions.py` is new** and runs in `exudev generate --all-checks` as the tenth
check: it names the file and the line of an `ABRV:` key the abbreviations list does not have. It
found the first one immediately - `\hac{ODE2t}`, twice in `itemFunctions.py`, a key that has never
been in the list, in a description that reaches no page, so nothing ever said so. Those two read
*"ODE2 time derivatives"* now.

Two more things came out of the conversion:

- **Three calls were written `\\hac{...}` in a non-raw string** - the doubled backslash is the
  writer paying for the escape by hand, which is the case RG3.14.9 is about. They are plain
  `ABRV:` now.
- **`\acf`, one of the seven, was never handled by the docstring cleaner.** `ObjectGenericODE1`'s
  docstring in `itemInterface.py` read *"a system of acf{ODE1}"* - the backslash-stripping pass
  removed the backslash and left the rest. It reads `ODE1` now, through the new
  `docstringText.StripAbbreviations`, which the pybind stub path calls as well. The wider leak in
  that path is recorded on **#2652** rather than fixed here, because the cleaner also rewrites
  `\refSection{...}` to the literal `theDoc.pdf`, a document that has not existed since D8.

`docs/generated` is **byte-identical**: the conversion is equivalent, which is the point.

<a id="rg3-14-2"></a>
### RG3.14.2 — a heading is the heading it becomes (2026-09-25, #2655)

A heading in a description was `\mysubsubsubsection{Equations of motion}`, and the level was hidden
twice over. The macro meant five `#`, `ConvertSections` emitted five, and `NormalizeHeadings`
quietly compressed the page's source levels **1, 3, 4, 5** to the rendered **1, 2, 3, 4**. Nothing
could be checked, because the number the writer wrote was never the number the reader saw.

The 135 headings are now written as what they become - `#### Equations of motion` in an item's
`equations` text, `## Title` in a structure's `latexText` - and `itemDocsEmitter` emits the item
and *DESCRIPTION* headings at their final levels too, so `NormalizeHeadings` has nothing left to
repair on an item page. A labelled heading carries its MyST target on the line above it.

`checkDefinitions` gained the check: **a heading at the wrong level, and a title that means one of
the ten recurring sections but is spelled differently.** It had four to find:

| found | against |
|---|---|
| *Connector Forces*, 3 times | *Connector forces*, 11 times |
| *PostNewtonStep* | *Post Newton Step*, 3 times |
| a heading whose `(classicalFormulation=True)` was commented out with a `%` | its sibling, which kept it - so two sections of one item had the same title |

One more thing came out of giving the emitter the real levels: **MINI EXAMPLE was a sibling of
DESCRIPTION rather than of Equations**, on all 23 item pages that have one. It is a `####` now.

The generated pages move by exactly **28 lines, each one a replacement**: the four corrected
titles and the 23 MINI EXAMPLE headings.

<a id="rg3-14-3"></a>
### RG3.14.3 — a reference is a Markdown link (2026-09-25, #2655)

The 144 references of `definitions/` were written in **nine** macros - `\refSection`,
`\refSectionA`, `\refChapter`, `\ref`, `\fig`, `\eq`, `\eqs`, `\eqq`, `\eqref` - which the
converter turned into a `{ref}` or an `{eq}` role with the same target name. They are

```
[](#sec-item-objectground)     [](#eq-objectground-position)     [](#fig-objectspheresphrecontact)
```

now, and the **empty text is the point**: the page supplies the heading, the equation number or the
figure caption, exactly as the role did. The ten figure labels are the MyST targets they already
became.

**Probed against Sphinx 9.1.0 / myst-parser 5.1.0 with this project's settings, before converting
anything.** Four target forms resolve from another page, with and without link text: a `(name)=`
above a heading, an equation label written `$$...$$ (name)`, a `{figure}` with `:name:`, and a
`(name)=` above a `{figure}`. A target above a **paragraph** resolves in nothing - not as
`(name)=`, not as an inline `{#name}`, not as a raw `<a id>` - and only `` {ref}`text <name>` ``
reaches it, with the text, because `` {ref}`name` `` alone warns *"A title or caption not found"*.
That is the whole reason RG3.14.1 keeps a macro for the abbreviations, whose list is exactly such a
list of paragraph targets. And it is the whole set: the 55 `\label`s of `definitions/` are **45
equations and 10 figures**, with every section label written as a `...sectionlabel` macro on its
heading.

The 45 equation labels stay inside `\be .. \ee` for now: the display math is the other half of this
step, and a label is converted with the delimiters that hold it.

**In a docstring there is no page to link to**, so `docstringText.PlainTextLinks` renders the link
as its own text. Eight docstrings improve by it, because `\refSection` used to be rewritten to the
literal `theDoc.pdf` and now names the section a reader can search for.

The generated pages move by 246 lines, each one a role replaced by the link that resolves to the
same target - and `exudev docs` under `-W` is what proves all 144 of them resolve.

<a id="rg3-14-10"></a>
### RG3.14.10 — a citation with no macro, and a field that says what it holds (2026-09-25, #2655)

Two questions from the maintainer, one after the other.

**"A citation needs no macro: so how are macros handled then? can they be checked?"**

The 31 `\cite{key}` calls of `definitions/` are `[ZwoelferGerstmayr2021]` now, and the generated
pages are **byte-identical** - because `\cite` already rendered as `[key]`, and the link is made by
`conf.py`, which appends a Markdown link definition for every key of `docs/bibliographyDoc.bib` to
every document Sphinx reads. So the macro was never doing the work.

It can be checked, but only in one direction, and the measurement says why. A key that **is** in
the bibliography resolves. A key that is **not** stays plain text and nothing fails - and no
pattern can tell a citation from the other square brackets a description is full of: measured over
`definitions/`, **1228 bracketed tokens**, of which `[0,0,0]`, `[SI:kg]`, `[localIndex]` and
`[exu.JointType.RevoluteZ]` are typical. Twelve of the 100 bibliography keys are not even shaped
like a key (`pybind11`, `EXUDYNgit`, `coumans2015bullet`), so a shape rule would miss them too.

What can be told apart is a **near miss**. Of the 38 bracketed word-like tokens in `definitions/`,
30 are keys, and the closest any of the other eight comes to a key is a similarity of **0.59**. A
cutoff of 0.85 therefore reports a typo - `[ZwoelfrGerstmayr2021]` is 0.98 from
`[ZwoelferGerstmayr2021]` - and reports nothing else. `checkDefinitions` does that, and says which
key was probably meant. A key that is nothing like a real one is still caught only by reading the
page, and `definitions/README.md` says so rather than promising more.

**"The latexText in StructureDefinition: the field name obviously has to change."**

It does: the field is neither LaTeX nor, since RG3.14.2, written in it. It holds the `##` heading
and the paragraph that open the section a group of structures forms - four of them, one per
`structureDefs*.py` file. It is **`sectionText`**, in the four files, in the template of
`structureModel.py` (whose comment said *"text, which will be added before the class description
(e.g., to start a new section)"* and now says what it holds), in `structureDocsEmitter`, in the
mangle rules of `itemModel` and in the heading-level table of `checkDefinitions`. The local
variable in `structureDocsEmitter` that carried the chapter's own introduction was called
`latexText` too, and is `chapterIntro`.

The regeneration is byte-identical.

<a id="rg3-14-11"></a>
### RG3.14.11 — the citation marker, and the last of theDoc.pdf (2026-09-25, #2655)

**The citation is `[CITE:ZwoelferGerstmayr2021]`** (maintainer, 2026-09-25): *"any left `[CITE:`
would already indicate that something went wrong. Clearly, other `[FunnyCitation]` would stay, but
this is ok."* The converter drops the marker and `conf.py` does the rest, so the 31 citations of
`definitions/` render exactly as before - the pages are byte-identical - and the marker buys three
checks that were impossible without it:

- a `[CITE:Key]` whose key the bibliography does not have, **exactly**, with the nearest key named;
- a key written **without** the marker - `[ZwoelferGerstmayr2021]` - which is the writer who forgot;
- a bracketed word that is nearly a key, for the writer who forgot both.

And a `[CITE:` left anywhere in a generated page is a conversion that did not happen; there is none.
`docstringText.PlainTextLinks` drops the marker too, for the day a cited description becomes a
docstring.

**`\refSection` now names its section.** `CleanStringForPyiDescription` rewrote it to the literal
`theDoc.pdf` - a document that has not existed since decision D8 - so a docstring told the reader to
open a file that is not there. It writes the target name, the same thing a native Markdown link
leaves behind, which is what a reader can search for. One generated docstring changes:
`ComputeODE2Eigenvalues` says *"see sec-mbs-systemdata"* instead of *"see theDoc.pdf"*.

Five **hand-written** docstrings in the shipped package still say `theDoc.pdf` in their own text, so
no regeneration reaches them; that is **#2656**.

**Eight `\refSection{...}` in the issue archive** (maintainer: *"I don't know why... could give a
bad rendering of the issue"*) are the plain section names now, in `archive/2021.json`, `2022.json`
and `2023.json`. They are historical issue texts from before the macro had any meaning outside the
LaTeX build, and they were published as raw macros in the tracker page. The two left are in the
working remarks of **#2652**, where the macro is the subject of the sentence.

**The stubtest backlog lost its OpenVR entry.** RG6.1 removed `VSettingsOpenVR` and the baseline
still listed it; `checkPython --stubs` said so on every run, because the backlog is meant to shrink.
272 entries now.

<a id="rg3-14-3-1"></a>
### RG3.14.3.1 — the display math (2026-09-25, #2655)

The 417 display formulas of `definitions/` were written `\be ... \ee` and `\bea ... \eea`, Exudyn's
own delimiters, and the converter turned them into the `$$ ... $$` that Markdown's display math
already is. They are written that way now, with the 45 equation labels in the MyST form
`$$ ... $$ (eq-name)`, so a reference to an equation - RG3.14.3 made those native links - points at
something the source shows. The mathematics itself did not change: it is LaTeX, and it stays LaTeX.

**The conversion was done by the converter.** `latexToMarkdown.ConvertDisplayMath` was applied to
the source block by block, so there is no second implementation of `\eqComma`, `\eqDot`,
`\nonumber` and the `&=&` of an aligned block to drift from the first. Two things had to be
arranged, and both are consequences of doing at the source what the pipeline did in the middle of
a run:

- `StripComments` runs **before** `ConvertDisplayMath`, so a `\be` on a commented-out line never
  opens a block. On the source it does, and it swallows everything up to the next `\ee` - which is
  what the first attempt produced. The 26 delimiters inside a LaTeX comment are hidden from the
  pass and put back after it.
- `RemoveIndentation2` dedents a description by its **minimum** indentation before the converter
  sees it, so `ConvertDisplayMath` emits its block at column 0. In the source the block keeps the
  indentation of the `\be` it replaces, or the minimum drops and everything that depends on it
  moves.

**And one pass was working by accident.** `ConvertRSTFigures` matched `.. _label:` and
`.. figure::` only at the start of a line, which held only because `RemoveIndentation2` had already
removed the indentation. The pattern allows it now. Without that, five figure targets came out of
the build as raw RST - `.. _fig-objectcontactconvexroll-sketch:` - instead of MyST targets, and the
`[](#fig-...)` links of RG3.14.3 pointed at nothing.

**The pages move by 13 lines, and every one is a repair.** A `\be` block inside a list item used to
be joined into the item's text line (#2593), which glued the sentence after the formula onto the
sentence before it:

> *"1. if gap $x_{gap,lastPNS}$ of previous `PostNewtonStep` had different sign to current gap, set
> while otherwise $\varepsilon^n_{PNS}=0$."*

That now reads *"...had different sign to current gap, set"*, the formula, *"while otherwise
$\varepsilon^n_{PNS}=0$."* Four sentences in `ObjectContactFrictionCircleCable2D` and
`ObjectGenericODE2` are put right this way.

<a id="rg3-14-6"></a>
### RG3.14.6 — the two RST switches, and two passes that were working by accident (2026-09-25, #2655)

`\onlyRST{..}` kept its content and `\ignoreRST{..}` dropped it, which made every figure in
`definitions/` a pair: an RST directive for the build and a LaTeX `figure` environment beside it
for a PDF that has not been made from these sources since decision D8. **All 13 `\ignoreRST` blocks
were text that reached no builder** - `grep includegraphics docs/generated` finds nothing - and they
are deleted. The 11 `\onlyRST` blocks are replaced by what `ConvertRSTFigures` and `ConvertRSTImages
` produced from them, so the pages keep their figures and the passes have nothing left to convert.
`ResolveRSTSwitches`, `ConvertRSTFigures`, `ConvertRSTImages`, `LatexRSTFigure` and
`DropLatexFigures` are gone with them; `latexToMarkdown.py` is 485 lines.

The one pair worth a decision, the kinematic-tree algorithms, was decided by RG3.8.2, which put the
algorithms into the text as numbered lists. There was no case left for keeping even one switch.

**Two passes turned out to be working by accident, and the second one was hiding lost text.**

`RemoveIndentation2` in `itemDocsEmitter` dedents a description by its **minimum** indentation, and
it counted a line that is nothing but spaces. Several item descriptions have a stray one-space
line, so the minimum was **1** where every real line is indented by 4 - the function printed
`minIndent= 1` on every run, which is the author's own note that something was odd. The text was
therefore dedented by one, everything downstream was off by three, and `ConvertRSTFigures` only
worked because it did not care. A blank line has no indentation; it is skipped now.

With the dedent right, `ConvertLists` showed what it had been doing. It collects an item's prose
into **one** line and puts formulas and sub-blocks after it, so the sentence that explains a result
was moved in front of the result:

> *"...had different sign to current gap, set while otherwise $\varepsilon^n_{PNS}=0$."*

and in the worst case the text was simply gone: the `PostNewtonStep` algorithm of
`ObjectContactFrictionCircleCable2D` is seven numbered steps, and the page showed **step 1 and the
formulas**, with step 2 glued into the parent bullet and steps 3 to 7 nowhere. Prose that follows a
formula now stays after it. That is the whole of the 11 lines the pages move by: **one line
replaced and ten restored.**

<a id="rg3-14-9"></a>
### RG3.14.9 — a description that carries a backslash is a raw string (2026-09-25, #2655)

Python reads `'\theta'` as a tab followed by `heta`. Nothing says so: the page shows the tab, the
formula is gone, and the build is green. The writers of `definitions/` had been paying for this by
hand - measured before the step, **158 literals held a doubled backslash**, `'\\item'`,
`' \\\\ \\\\ Usage:\n\\bi\n'` - which works and is unreadable, and 25 more carried a `$` in a
non-raw literal, one `\n` away from the same accident.

All 183 are `r"""..."""` now, and **the value did not move**: the rewrite evaluates each string
expression with `ast` and writes the same value back as one raw literal, which was verified by
comparing every string value of every call before and after, and then by the regeneration being
byte-identical. A chain of one-line strings joined by `+` - which is how the `pybind*.py`
descriptions were built, six of them with `\\` escapes on every line - becomes the multi-line text
it always was.

`checkDefinitions` now rejects a literal whose value carries a backslash or a `$` and which is not
written `r'...'`. It is deliberately a rule about **the source text**, not about the value: by the
time a wrong escape is a value, the evidence is gone.

<a id="rg3-14-4"></a>
### RG3.14.4 — a table is a Markdown table (2026-09-25, #2655)

The 48 tables of `definitions/` that are not a user function's argument list are Markdown pipe
tables now, written where they stand in the text, and the generated pages are **byte-identical**:
the conversion is the converter's own `ConvertTables`, applied in place. The 34 argument tables stay
for RG3.14.5, which turns the whole user function block into a typed Python signature; four more
`\startTable`s are commented out and therefore already dead.

**Why the pipe table and not the dict.** The plan proposed `quantities=[Quantity(name, symbol,
description)]` in the item's dict for the 39 *"Definition of quantities"* tables, which the
maintainer had asked for: *"there could be just an additional structure in the definition of an item
which contains a list per row"*. Measured before doing it: **32 of the 39 open the `equations` text,
and 7 do not** - three items carry a second one further down, and in `MarkerSuperElementRigid` the
table sits under a sentence that introduces it. A field in the dict says *what* the table holds but
not *where* it goes, so those seven would move on the page, or the dict would need a placement
marker in the text - more machinery than the table itself. **A pipe table says both, in one place,
and it is native Markdown**, which is what the hand-written chapters use. If the symbols should
become data later - so that a checker can say whether every symbol is a declared math macro - that
is a step of its own, and it is worth doing for the symbol column alone rather than for the table.

Two things had to be arranged so that nothing was lost:

- **A LaTeX comment inside a table block is kept, above the table.** `StripComments` runs before
  `ConvertTables`, so a commented-out `\rowTable` is not a row - `ObjectRigidBody` has two, and a
  first attempt that replaced the block wholesale both put them into the page and deleted them from
  the source. 33 comment lines are kept.
- **The work is done per string value, through `ast`, never on the file text.** A table pattern let
  loose on a file matches from one item's `\startTable` to the *next* item's `\finishTable`: the
  first attempt gave `ObjectRigidBody` two rows belonging to `ObjectRigidBody2D`.

<a id="rg3-14-5"></a>
### RG3.14.5 — the user function blocks, and the last LaTeX out of definitions/ (2026-09-25, #2655)

A user function was four macros: `\userFunction{sig}`, the `\startTable{arguments / return}` of its
arguments with `\returnValue` as the last row, `\userFunctionExample{}`, and a `lstlisting` block.
All 35 blocks are Markdown now - a bold line naming the signature, a pipe table, *Example*: and a
fenced `python` block - and the conversion is the converter's own passes applied in place, so the
pages keep every word.

**`definitions/` is free of structural LaTeX.** `\userFunction`, `\returnValue`, `\startTable`,
`\rowTable`, `\finishTable` and `lstlisting` are at zero, except in four tables that are commented
out and were therefore never rendered.

**The typed Python signature was measured and not used.** The plan, from the maintainer's own
suggestion, was

```python
def forceUserFunction(mbs: MainSystem, t: Real, itemNumber: Index,
                      q: 'Vector $\in \Rcal^{n_{ODE2}}$', ...) -> 'Vector6D':
```

and the obstacle is in that line: **35 of the 228 argument rows say the *size* of an argument as a
formula** - `Vector $\in \Rcal^{n_{ODE2}}$`, `MatrixContainer $\in \Rcal^{(n_{q_{m0}}+n_{q_{m1}})
\times n_{ae}}$` - and a formula does not render inside a code block. Putting the arguments into a
signature would turn 35 sizes into literal LaTeX on the page. The table renders them, so the
arguments stay a table; what the signature idea was really for - the `q\_t` escapes that exist only
because the text was LaTeX - is gone either way, because the signature is now written as the Python
it is.

The pages move by **37 lines, every one an added blank line**: `**Userfunction**:` and `*Example*:`
were glued to the end of the paragraph before them, because a macro replaced inside a paragraph
stays inside it. Each of them starts its own paragraph now.

Two rules from the earlier sub-steps had to be applied again, and a third was learnt here: only a
string value that **holds** one of the constructs is touched at all. The newline collapse that
`ConvertText` ends with is right for a description and wrong for a `miniExample`, whose blank lines
are the Python's own - an attempt that collapsed every value rewrote the mini examples of five files.

<a id="rg3-3-1"></a>
### RG3.3.1 — the PDF is built without Perl (2026-09-25, #2658)

The maintainer: *"I just observed that the PDF docs creation currently does not work."* It worked
here, twice, which was the useful part of the puzzle - the same repository, the same MiKTeX, one
machine, two answers. Their output named the cause:

> `MiKTeX could not find the script engine 'perl' which is required to execute 'latexmk'.`

**`latexmk` is a Perl script.** Git for Windows ships a perl in
`…\Programs\Git\usr\bin`, Git Bash puts that directory on PATH and PowerShell deliberately does not,
and every run of mine went through Git Bash. So `exudev docs --pdf` never depended on the TeX
installation alone: it depended on which shell started it, and nothing said so.

`latexmk` automates three things - run the engine, build the index, run the engine again until the
cross-references stop moving - and `commands.BuildDocumentationPdf` does them with the engine and
`makeindex` that every TeX installation brings:

- `xelatex -interaction=nonstopmode`, plus `--enable-installer` on Windows so that MiKTeX fetches a
  missing package instead of opening a dialog nobody is there to answer;
- `makeindex -s python.ist` after the first pass, and an **empty `.ind` for an empty `.idx`** -
  `makeindex` refuses an empty input and the document needs the file to exist, which is exactly what
  the `latexmkrc` sphinx writes does in its own `xindy` wrapper. This document's `.idx` **is** empty;
- then another pass, and another while the engine asks for one **or the files it reads on the next
  pass have changed**. That second condition is the one that matters: the first version asked the
  `.log` only, and one of its markers was *"There were undefined references"* - a statement about the
  document, not a request. This document had 34 of them, so the marker never cleared and every build
  ran to the five-pass limit. latexmk decides by the contents of the `.aux`, `.toc`, `.out` and
  `.idx`, and so does this now.

**Verified with every directory holding a `perl.exe` removed from PATH** - the maintainer's
situation, reproduced rather than imagined: 1103 pages, 10.3 MB, **two passes, 54 s**, where latexmk
took 75 s and the first version of this 2 m 18 s.

And the 34 undefined references were not noise. See RG3.14.3.2: they were the reason this step had to
come first.

<a id="rg3-14-3-2"></a>
### RG3.14.3.2 — a reference to an equation is the role, not a link (2026-09-25, #2655)

RG3.14.3 turned all 144 references of `definitions/` into native Markdown links, and the HTML build
under `-W` said they all resolve. **42 of them point at an equation, and every one of those left the
PDF as an undefined reference.** The LaTeX writer gives a link to an equation the anchor

```
docs/generated/items/NodeRigidBodyEP:equation-eq-noderigidbodyep-gm
```

while it labels the equation itself

```
equation:docs/generated/items/NodeRigidBodyEP:eq-noderigidbodyep-gm
```

- the word and the separator in different places. The `{eq}` role, which the manual chapters still
use, writes `\eqref{equation:…}` and matches. So for an equation the role is what works and the link
is not, and the 42 references are roles again. Sections, chapters and figures stay native links:
none of those was undefined.

Two things worth keeping in mind from this. The HTML build is **not** sufficient evidence that a
reference resolves - the two writers disagree, and only the PDF said so, which is why RG3.3.1 had to
be fixed first to be able to see it at all. And `checkDefinitions` now rejects a Markdown link whose
target is one of the 44 equation labels, so the form cannot come back by hand.

<a id="rg3-15"></a>
### RG3.15 — the chapters of the user manual (2026-09-25, #2657, #2661)

The maintainer's table of contents, carried out. What was eleven visualization sections in *Exudyn
basics*, five in the graphics chapter and one in *Advanced topics* is one chapter in four sections:

```
Renderer, graphics and visualization
    The renderer window        <- Renderer and 3D graphics, Mouse input (+6D mouse), Keyboard input,
                                  Visualization settings dialog, Execute command and help
    The model view             <- Render state, Storing the model view, Camera following objects
    Images, animations and     <- Solution viewer, Storing images and generating animations
        the solution viewer        (+ Software rendering, Generating animations)
    How to add graphics        <- Graphics user functions via Python, Color RGBA and
                                  alpha-transparency, Character encoding: UTF-8
```

**Performance, errors and solver failures** is a new chapter, `docs/manual/performanceErrors.md`,
with the three sections that were the end of *Exudyn basics*: the errors Exudyn raises with their
five sub-sections, removing convergence problems, and the ways to speed a model up.

**Advanced topics** took what is internals or reference rather than use - the graphics pipeline,
raytracing and the whole `GraphicsData` reference with its six sub-sections - and, for #2661, *the
command line* and *the results monitor*, which were chapters of their own between the manual and the
notation. They are sub-pages of a new section, *Tools that are not part of a model*.

**Exudyn basics** keeps one visualization section, *Seeing the model*: the four lines that start and
stop the renderer, and a link. Its introduction listed six topics of which four had left, so it says
what the chapter now holds and points at the two chapters that took the rest.

**The move was done by a tool, not by hand**, and that is what makes it safe. A page is read as a
list of sections - a heading, the MyST targets standing above it, and the text down to the next
heading, sub-sections included - and the new pages are assembled from those sections; a section
therefore carries its targets with it and every reference to it keeps working. **Measured: 39 targets
before, 46 after, none lost** - the seven new ones are the four section headings, the new chapter, the
new *Seeing the model* and the tools section. `exudev docs` under `-W` passes, which is the second
half of the proof: a reference that had lost its target would fail the build.

One thing is left for the maintainer to decide: **the file is still called `GUI.md`** while the
chapter it holds is called *Renderer, graphics and visualization*. Renaming a tracked file needs
approval, and it changes the URL of the page.

<a id="rg3-16"></a>
### RG3.16 — every heading in sentence case (2026-09-25, #2662)

The maintainer, reading the new table of contents: *"there is no unified headings style upper/lower
case: use 'This is a heading' style for all."*

Sixteen headings are renamed, and they are listed one per line in the log of the commit so that each
can be judged: *Installation and Getting Started*, *Exudyn Basics*, *Execute Command and Help*,
*Generating Animations*, *Parameter Variation*, *Genetic Optimization*, *Modeling of Contact in
Exudyn*, two *contact: Equations*, *Dynamics: Mechanical principles*, *Generalized Principle of
Virtual Work*, *Generalized Forces*, *Lagrange's Equations of Motion*, *Euler's and Chasles's
Theorems*, *Install from specific Wheel*. The last one gained an article as well, because *"Install
from a specific wheel"* is the sentence it was trying to be.

The sixteenth is a different kind: `ARCHITECTURE.md`'s *"The three-fold split: C / Main /
Visualization"* names the three class prefixes of the C++ code, so they are written as code -
`` `C` / `Main` / `Visualization` `` - which is both more correct and what the new checker reads.

**`tools/checkHeadings.py` is the eleventh check of `exudev generate --all-checks`.** A heading is
sentence case unless a capital in the middle is one of three things: a **proper noun** from a list
(*Newmark*, *Runge-Kutta*, *Hurty-Craig-Bampton*, *Ubuntu*, *Microsoft*, ...), a **name the code
spells with a capital**, which is recognised by its shape - CamelCase, ALLCAPS, or a name with a
digit - rather than listed, because the code has hundreds of them, or a word inside `` `code` `` or
`$math$`, which is not prose. Two more rules came out of the first run: what follows a colon after a
code name are that name's **values** and are spelled the way the code spells them (*GraphicsData:
Line*), and a leading marker is not the first word (*"(A) Solve for unknown accelerations"*).

All **353** headings of `docs/manual/`, `docs/dev/`, `docs/howTo/`, `index.md` and `CONTRIBUTING.md`
pass it.

<a id="rg3-17"></a>
### RG3.17 — a comment in a description is an HTML comment (2026-09-25, #2663)

The maintainer: *"In the itemDefs... files I still see `%` used for comments - latex style. Shouldn't
we use `<!-- This is a single-line comment -->` comments? `%` could still survive as the `%` symbol,
but maybe just inside a formula."*

That is what it is now. **437 comments in 263 blocks** are `<!-- ... -->`, a run of consecutive comment
lines being one comment - which is what a commented-out table wants to be - and `StripComments`
removes them before anything else reads the text. Four spellings are left exactly as they were:

- the **16 percent signs inside mathematics**, where MathJax and LaTeX are the ones that read them.
  The new `StripMathComments` removes such a line from the *output* so that dead LaTeX does not
  travel into the published page, but the source keeps it, and a writer commenting out a line of a
  formula writes what a LaTeX author would write;
- the **134 `%%RSTCOMPATIBLE`** lines, which are not comments but the marker `itemDocsEmitter` splits
  the published part of a text on;
- the one **escaped percent sign**;
- everything in a value that is **Python rather than prose** - a `miniExample` has `#10% stretch` in a
  comment of its own code.

**The reason to change it is in `StripComments`**: it runs *before* the mathematics is protected, so a
`%` it took for a comment truncated the rest of its line whatever that line was. That is not
hypothetical - it is why the *marker velocity* row of `MarkerSuperElementRigid` has been published
with its formula cut off: a `%` between two `$...$` segments of the row ended the cell.

Two comments could not be classified automatically, and they are the same case: they sit between two
`$...$` segments of what RG3.14.4 had made a single table-row line, and the number of dollar signs
before them is odd, so the measuring pattern took them for mathematics. Converted by hand.

`checkDefinitions` gained the rule: **a `%` outside mathematics is an error**, with the `%%RSTCOMPATIBLE`
marker, an escaped percent sign and a `%` that already sits inside an HTML comment as its three
exceptions.

The pages move by **15 lines, all of them a blank line that is now there**: a comment-only line leaves
a blank line, consistently - the old pass dropped the line when the `%` stood at column 0 and left a
blank when it was indented, which is why a bold lead-in like `**Userfunction**:` was sometimes glued
to the paragraph before it and sometimes not. It always starts its own paragraph now.

<a id="rg3-14-7-1"></a>
### RG3.14.7.1 — the macros that are a single element (2026-09-25, #2655)

RG3.14.7 in one pass moved 738 lines of the pages, so it is six sub-steps now, ordered by how much a
macro interacts with the line structure - which is where every failure has come from. This is the
first: 96 macros that stand for one thing and touch no line.

| macro | how many | becomes |
|---|---|---|
| `\codeName` | 11 | `Exudyn` |
| `\vspace{x}` | 20 | nothing |
| `\noindent` | 17 | nothing |
| `\paragraph{x}` | 15 | `**x**` |
| `\footnote{x}` | 13 | ` (x)` - what Markdown offers here |
| `\text{x}`, `\mysmall`, `\textdegree`, `\phantom{x}`, `\exuUrl{u}{n}` | 1 each | `x`, nothing, `°`, nothing, `[n](u)` |
| the escaped space | 55 | a space |

The escaped space is the maintainer's own note: *"`\codeName\ ` was used because of following ',' and
similar symbols"* - the backslash-space ended the macro name where a comma follows, and it is a space.

**The published pages are byte-identical.** What moved is 114 lines of `src/Autogenerated` and the
shipped docstrings, where a description is embedded raw - and every one of them is a repair, because
the docstring cleaner removes a backslash and leaves the word: `ObjectJointRevoluteZ` said *"this
avoids 180textdegree flips"* and says *"180°"* now.

**Two traps, both from converting at the source what the pipeline does in the middle of a run.**

`ConvertInline` turns `\\` into a hard line break *before* it turns an escaped space into a space. A
pass that does only the second one matches the **second backslash of `\\ `** and takes the line break
apart - which is how the first attempt joined *"...from Python to C++."* and *"Usage:"* into one line.
The escaped space is matched with a negative lookbehind for a backslash now, and the line break
itself is RG3.14.7.5.

And a pass must not lift a comment or a code block out of the text behind a control character:
the placeholder survived this pass and not the next one, and `#10% stretch` in a mini example came
out as `#10 13`. There is no need for it - a macro inside a comment may as well be converted, since
the comment never reaches a page.

<a id="rg3-14-7-2"></a>
### RG3.14.7.2 - inline code is a Markdown code span (2026-09-25, #2655)

723 of the 1005 macros were one: `\texttt{x}`, which is now ``x``. The LaTeX escapes inside it -
the `\_` and `\&` that only existed because the text was LaTeX - go with it, which is what ConvertInline
did with them.

**The published pages are byte-identical**, and one regression had to be repaired to make them so.
A docstring is RST, not Markdown: `docstringText.CleanStringForPyiDescription` turned `\texttt{x}`
into the RST literal ```x```, and with the source in Markdown it would have passed a single backtick
through - which in RST is a title reference and not code. It converts a Markdown code span to the
RST literal now, so every docstring is what it was.

What moved is 98 lines of `src/Autogenerated` and one shape in the stub: a Doxygen comment that
carried `\texttt{MarkerNodeODE1Coordinate}` carries ``MarkerNodeODE1Coordinate`` , and three stub docstrings
lost their `r` prefix because they no longer hold a backslash.

<a id="rg3-14-7-3"></a>
### RG3.14.7.3 - the bold forms (2026-09-25, #2655)

76 macros, and the pages and the generated sources are **byte-identical**: 47 in the brace shape
`{\bf x}` that the item definitions use, and 29 `\mybold{x}`, both of them `**x**`. There was
no `{\it x}` left to convert and no `\textbf` or `\textit` either, which the converter also handled -
four of its cases were dead.

What is left of the 1005 is **146 in 7 names**: the lists with the line breaks that sit between them
(RG3.14.7.4), six `\addExampleImage`, and in the pybind files six `\tabnewline` of an example that used
to be a table cell (RG3.14.7.5). One census entry was an artefact: the four `\n` of
`pybindMainSystem` are C++ string escapes in a lambda passed as `cName`, which is code and not a
description - so the gate has to know that keyword too.

<a id="rg3-14-7-4"></a>
### RG3.14.7.4 - the lists, and the line breaks between their items (2026-09-25, #2655)

The maintainer asked for the two together, and they belong together: **37 of the 41 line breaks
outside mathematics stand at the end of an `\item` or in the middle of one**. 20 lists, 84 items and
42 breaks, and what is left of the 1005 macros is **18 in 5 names**.

A LaTeX line break is not two trailing spaces - the maintainer decided that: *"in latex, these
spaces ment nothing, so why should they do here"*. Between paragraphs it is a blank line; inside what
becomes one list item it is a **continuation paragraph** of the item. The 112 breaks inside
mathematics are row separators of an array or an aligned block and are untouched.

**The pages move by 72 lines and every one is one of those two things**: a trailing double space
that was a forced break is gone, and where the break carried meaning it is a paragraph break -
`- **CASE SN**: use **S**egment **N**ormals` now stands on its own line above the sentence that
explains it, instead of being welded to it.

Three things had to be arranged, and each of them was a wrong page first.

- `ConvertLists` emits a list at **column 0** whatever the indentation of its input, and indents a
  line that is already indented inside a fenced block by two more. In the pipeline it runs after
  `RemoveIndentation2` has dedented the description, so column 0 is right; here each block is
  dedented before the pass and indented back after it - the same rule the display math of
  RG3.14.3.1 and the tables of RG3.14.4 needed. Without it **every figure inside a list item moved
  four spaces right**.
- A break inside an item **cannot** be a blank line: `ConvertLists` joins an item's own lines into
  one and drops it. It is carried through the join as a marker and split out afterwards.
- A list may open **in the middle of a line** - `Usage: \bi` in four pybind descriptions - so the
  indentation is read from the line, not from the macro.

What is left: six `\addExampleImage`, six `\tabnewline` of the one pybind example that used to be a
table cell, and the `\n` family, which is C++ in a `cName` lambda and not a description at all.

<a id="rg3-14-7-5"></a>
### RG3.14.7.5 - the last eighteen, and a marker that was not a marker (2026-09-25, #2655)

**`%%RSTCOMPATIBLE` is not a relic; it is the switch that publishes an item's description.**
The plan said to remove the 67 markers, and removing them deleted **4221 lines of the pages**. The
reason is in `itemDocsEmitter`: the emission sat inside `if '%%RSTCOMPATIBLE' in eqText`, so the
marker did two things at once - it said where the published part ended **and whether there was one
at all**. The whole text is published now, unconditionally, and the markers are gone with the dead
LaTeX that stood after them.

**Two items got their description back**: `ObjectContactCircleCable2D` had written a *Connector
equations* section and `ObjectJointPrismatic2D` a *Geometric relations* section of thirteen lines,
and neither was on its page, because neither text carried a marker. Nobody had a way to notice.

**The two examples the maintainer pointed at.** `\tabnewline` was deleted by the converter, so the
example of `visualizationSettings` read

    EXAMPLE:SC = exu.SystemContainer()SC.visualizationSettings.autoFitScene=False

They cannot become a fenced block, which is what the maintainer expected: both are rendered into
**one line** - one as a bullet of the C++ interface page, one as a table cell - and
`PyLatexRST.MarkdownEntry` replaces every newline of a description by a space. So each is a code span
with `;` where the breaks were: *EXAMPLE: `SC = exu.SystemContainer(); 
SC.visualizationSettings.autoFitScene=False`*.

The six `\addExampleImage` are the `{image}` directive the converter wrote from them, and the one
`\\` that RG3.14.7.4 could not see - it sits against a word, `\\However` - is a paragraph break.

What is left in `definitions/` is **five backslashes in one place**: the `\n` of a C++ lambda passed
as `cName` in `pybindMainSystem`. That is code, not a description, and RG3.14.7.6 has to know the
keyword rather than convert it.

<a id="rg3-14-7-6"></a>
### RG3.14.7.6 - the gate (2026-09-25, #2655)

**A backslash command in a description, outside its mathematics, is an error.** That is the rule
the whole of RG3.14 was for, and it is the sixth in `tools/checkDefinitions.py`. Until now a macro
the converter did not know was carried through to the page as itself, and nothing said so - a
`ReportUnknown` that would have found them sat in `latexToMarkdown` and was **called by nothing**; it is
deleted, and the file is 490 lines.

What the gate does not look at, because it is not a description:

- a value passed to one of the code keywords. Two were found by writing the check: `cName` holds a
  C++ lambda whose `\n` is a string escape, and `addConstructor` holds C++ with a literal `\n` that the
  old parser turned into a newline. `cplusplusName` and `cppText` are in the list for the same reason;
- an HTML comment, which never reaches a page;
- a fenced code block;
- the mathematics itself, where `checkMathMacros` is the check and `\%` is the engine's own comment.

**RG3.14.7 is closed: 1005 macros, none left.** `definitions/` is Markdown with LaTeX mathematics,
and it stays that way now by a check rather than by attention.

<a id="rg12-4-1"></a>
### RG12.4.1 - a user function is a Python function (2026-09-25, #2664)

The maintainer rejected the string the plan had proposed: *"I wanted it ... not to be given in a
string, but defined in the Python code ... because this avoids problems in the definition itself and
immediately becomes Python"*. It is a real `def` now, and this sub-step is the vocabulary that
makes one possible.

**In `definitions/definitionTypes.py`**: the names an annotation is written with - `Real`, `Index`,
`Bool`, `MainSystem`, `BodyGraphicsData`, `MatrixContainer`, `ConfigurationType` and numpy for
`np.ndarray`. They are names, not re-implementations: the real `MainSystem` is in the compiled module.
They have to exist because `definitionLoader` **imports** a definition file, so an annotation is
evaluated - and because `ruff` lints only `python/exudyn`, nothing in the gate would have caught an
undefined name here. `ItemParameter` takes `userFunction=`, the function itself.

**In `tools/generators/userFunctionModel.py`**: the reader. `ReadUserFunction` returns the argument
names with their annotations **as written**, the return annotation, the summary, and the `Args:` and
`Returns:` lines of a Google-style docstring - the same shape `docstringText` already parses for the
stub files, so a writer meets one convention and not two. **Nothing is executed**: the function
object is used only to find its source, which is read with `ast`, so `np.ndarray` stays `np.ndarray`.
It raises where a docstring describes an argument the signature does not have.

The `def` is named `<Item>_<parameter>`: one definition file holds 35 user functions and four of them
are `forceUserFunction`, which at module level would shadow each other in silence. The name the
documentation prints is the parameter's `pythonName`, so that is not visible on a page.

It reads **0 user functions** today, which is the point: nothing changes until a definition carries
one. RG12.4.2 is the first, and it needs one decision first - **where a generated block goes**. The
four user function blocks of `ObjectGenericODE2` sit at the end of its equations text, but they are
followed by an `*Example*:` and a code block that belongs to the first of them, so generating them at
the end would reorder the page. An item with **one** user function and no trailing example -
`ObjectGround.graphicsDataUserFunction` - is the cleaner first end-to-end test.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.2 - one user function, end to end (2026-09-25, #2664)

`ObjectGround.graphicsDataUserFunction` is now an ordinary Python function in
`definitions/itemDefsObjects.py`, written above the `definitions.append(...)` of the item and handed
to its `ItemParameter` by object. The 48 lines of hand-made block that stood in the item's
`equations` - the bold signature line, five lines of prose, a three-row argument table, a separator
and a fenced example - are gone; what remains of them is a two-line signature with real annotations,
a Google-style docstring, and the example as `userFunctionExample=r'''...'''`.

**The page is the proof.** `docs/generated/items/ObjectGround.md` differs from the committed one in
**one line**: a blank line that sat inside the example's code fence, because the source had a blank
line before the closing fence. Everything else - the wording, the table, the double space in
`arguments /  return`, the position of the block after the equations - is byte-identical, generated
from the def instead of copied from prose.

**What the mechanism is.** Four small pieces, none of which does anything to an item that carries no
def:

- `tools/generators/userFunctionModel.py` reads the def: `inspect.getsource` for the text, `ast` for
  the tree. Its `_SplitDocstring` was corrected here - the summary is the **first line** and the
  details are what follows it, with their own line breaks kept, because a description is Markdown and
  a writer laid those lines out. A paragraph-joining summary would have rewrapped the five lines into
  one and made the comparison meaningless.
- `definitionLoader._Member` carries the **function object** through to the emitters, beside the
  strings the old line parser produced.
- `itemDocsEmitter.UserFunctionDocumentation` builds the block as *definition-style Markdown* and
  sends it through the same `ConvertText` as a hand-written description, so `[](#sec-graphicsdata)`,
  a formula in a type cell and an `ABRV:` all behave the same in it.
- `itemInterfaceEmitter.CreateStringSymbolicUserFunctionArgs` takes the argument **names** from the
  def where there is one, and checks their **count** against the `std::function` of the parameter's
  type. That is the arity half of RG12.4.3, done here because this is the one place that holds both
  the def and the C++ signature. The single line it changed in `python/exudyn/itemInterface.py` is
  `['mbs', 'arg0']` becoming `['mbs', 'itemNumber']`; those names are read only by
  `advancedUtilities.ConvertFunctionToSymbolic`, and only to print the signature a user got wrong.

**The rules are in one place.** `definitions/README.md` section *Writing a description* now says that
a user function is not written in a description at all, with this def as the example, and says the
three things a writer needs to know: the first line is the summary, the size of an argument goes in
its `Args:` line as a formula, and the def is named `<Item>_<parameter>` while the page prints the
parameter's `pythonName`. Rule 6b of `CLAUDE.md` already points there.

**Gates**: 11/11 checks, the wheel, the full suite (`PASSED: no reproducible test failed`), and the
strict HTML build. The two drifts are the intended ones - `itemInterface.py` (tier 1) and the
`ObjectGround` page.

One user function of 23 is converted. RG12.4.4 - a `Protocol` in `itemInterface.py` - is now worth
doing on this one before the remaining 22 of RG12.4.5, because a Protocol is what a user actually
feels in an editor.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.3 - the check between the Python def and the C++ user function (2026-09-25, #2664)

The signature of a user function was stated in five places and compared in none. There is now one
comparison, `userFunctionModel.CheckAgainstCpp`, between the def of the definition file and the
`std::function` that `definitionTypes.userFunctionSignatures` maps the parameter's type to. It
reports four kinds of disagreement:

| finding | example |
|---|---|
| the number of arguments | `takes 1 argument(s), the C++ user function 2: MainSystem, Index` |
| an argument's type | `argument itemNumber: 'Real' cannot be the C++ Index (Index)` |
| the return type | `the return: 'BodyGraphicsData' cannot be the C++ StdVector (np.ndarray)` |
| an argument the docstring does not describe | `its row of the table would be empty` |

All four were provoked on purpose against the real `PyFunctionGraphicsData` and
`PyFunctionVectorMbsScalarIndex2Vector` signatures, and the last one is in because an undescribed
argument does not fail anything - it simply leaves a cell of the generated table empty.

**A size is deliberately not compared.** `cppToAnnotation` maps `StdVector`, `StdVector2D`,
`StdVector3D`, `StdVector6D`, `StdMatrix3D`, `StdMatrix6D`, `NumpyMatrix` and `StdArrayIndex` all to
`np.ndarray`, because the size of an argument is a formula in its description and not part of its
type - the maintainer's own correction, and the thing that makes a typed signature possible at all.
`py::object` says nothing about what it carries, so it accepts the three things that are passed as
one: `BodyGraphicsData`, `MatrixContainer` or `np.ndarray`. A C++ type that is not in the table is
itself a finding rather than a silent pass, so the vocabulary cannot grow behind the check's back.

**Two places call it, and that is on purpose.** `itemInterfaceEmitter` raises, because it cannot emit
`userFunctionArgsDict` from a def it does not believe; `tools/checkDefinitions.py` reports the same
findings with the file and the line, which is what a writer wants:

    definitions/itemDefsObjects.py:41  ObjectGround.graphicsDataUserFunction: the def argument
    itemNumber: 'Real' cannot be the C++ Index (Index)

That line is from a deliberately wrong annotation; `--check` exited 1 and the file was put back.

`CppSignatureTypes` reads the `std::function<...>` text and reports `const MainSystem&` as
`MainSystem`: a reference and a const are how C++ takes an argument and say nothing that a Python
annotation could state.

**Gates**: 11/11 checks, regeneration a no-op (the check changes no output), and the definitions
checker reporting the fifth of its families. Neither the wheel nor the test suite is touched: no
generated file changed, so what was built and tested in RG12.4.2 is still what is installed.

Two of 23 signatures' worth of machinery is now in place; what is missing for a user is RG12.4.4,
the `Protocol`.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.4 - the Protocol, which is the part a user feels (2026-09-25, #2664)

`python/exudyn/itemInterface.py` now carries a `Protocol` per user function that is written as a def,
before the classes that use it:

```python
class ObjectGroundGraphicsDataUserFunction(Protocol):
    """A user function, which is called by the visualization thread in order to draw user-defined objects.

    Args:
        mbs (exudyn.MainSystem): provides reference to mbs, which can be used in user function ...
        itemNumber (int): integer number of the object in mbs, allowing easy access
    Returns:
        list: list of ``GraphicsData`` dictionaries, see Section sec-graphicsdata
    """
    def __call__(self, mbs: exudyn.MainSystem, itemNumber: int) -> list: ...
```

**The annotation on the parameter is the half that matters.** A Protocol a user never names does
nothing for them, so `VObjectGround.__init__` states it:

```python
def __init__(self, show = True,
             graphicsDataUserFunction: Union[ObjectGroundGraphicsDataUserFunction, int] = 0, ...)
```

The `Union` with `int` is not decoration: `0` is the value that means *no user function*, and typing
the parameter as the Protocol alone would make the default a type error in the generated file. This
is the first annotation of any kind in a generated item constructor; the other parameters have none.

The docstring is rendered by the same `GoogleDocstringRenderer` and cleaned by the same
`CleanStringForPyiDescription` as every other docstring of the file, so a Markdown link reads as its
anchor and inline code as an RST literal, exactly as in the rest of the module. The runtime types come
from `userFunctionModel.annotationToPython`: `MainSystem` is `exudyn.MainSystem` - the class in the
compiled module, not the annotation name of the definition file - and an annotation missing from that
table stops the emit.

**One test model had to change, and finding it was worth the step.**
`python/TestModels/parameterConversionTest.py` walks `inspect.getmembers(itemInterface,
inspect.isclass)` and creates every class whose name starts with `Object`, `Node`, `Marker` ... -
which now includes `ObjectGroundGraphicsDataUserFunction`, an item type `GroundGraphicsDataUserFunction`
that does not exist. The suite said so at once (`1 TestModel TEST(S) OUT OF 126 FAILED`), and the
model now skips a class whose `_is_protocol` is true. Any generated type that is not an item will meet
the same walk, so the skip is the general fix and not a patch for this one class.

`__all__` needed nothing: `publicApi.PublicNames` reads the emitted source, so the Protocol is
exported by the same rule as everything else - which is also why the diff of `__all__` is large while
one name was added.

**Gates**: 11/11 checks, the wheel, the full suite (`PASSED: no reproducible test failed`), pytest
(404 passed, 2 skipped) and the strict HTML build.

**Not yet said to users.** `definitions/README.md` tells a developer that the Protocol exists; the
manual does not, because one user function of 23 has one and "annotate your function with its
Protocol" would be wrong for the other 22. It is said once they all have one - RG12.4.6.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.5.1 - the vocabulary of sizes, and the first five items (2026-09-26, #2664)

Six user functions of 23 are defs now: the four loads and `SensorUserFunction`.

**The size of an argument is in two places, and both are right.** RG12.4.2 mapped every array type to
`np.ndarray` on the grounds that a size is a formula in the description. That is true of a size that
is not fixed, and false of one that is: the argument tables say `Vector3D`, `Vector6D`, `Matrix3D`,
`Array` - 34 pages of them - and the C++ signature needs exactly that distinction, because
`StdVector3D` is not `StdVector`. So:

- a **fixed** size is part of the annotation. `definitions/definitionTypes.py` has `Vector`,
  `Vector2D`, `Vector3D`, `Vector6D`, `Matrix3D`, `Matrix6D`, `NumpyMatrix` and `Array`, all
  `np.ndarray` at runtime and all distinct as written; `cppToAnnotation` is now **one to one**, which
  is what RG12.4.7 needs to derive the `std::function` from the def.
- a size that is **not** fixed is the leading formula of the argument's description:
  `q: $\in \Rcal^n$ object coordinates ...` prints as `| q | Vector $\in \Rcal^n$ | object
  coordinates ... |`. The rule is `$\in ` and nothing else, because `$\fv$ copied from object` is a
  **symbol**, not a size, and belongs in the description where it already was.

**The conversion is done by a script that refuses.** It parses one `**Userfunction**` block out of an
`equations` string, builds the def from the table, wires the parameter through the tree - three loads
have a `loadVectorUserFunction` and only one of them belongs to the item - and stops if anything is
unfamiliar. It refused `LoadMassProportional`, whose block ends in the sentence *"Example of user
function: functionality same as in `LoadForceVector`"* instead of an example; by hand, that sentence
became a line of the details, which is the one place prose belongs, and it moved eight lines up the
page.

**What changed on the pages**, and nothing else did:

| change | where |
|---|---|
| `arguments /  return` -> `arguments / return` | the generated header; 21 hand-written tables have two spaces and 3 have one, and a generated one is the same everywhere |
| `exudyn.ConfigurationType` -> `ConfigurationType` | `SensorUserFunction`; the annotation is a name, and the module prefix is not part of it |
| one blank line inside the example's code fence | four pages, as in RG12.4.2 |
| the sentence moved above the table | `LoadMassProportional` |

Five `Protocol` classes and five annotated constructors came with them -
`LoadForceVectorLoadVectorUserFunction`, `SensorUserFunctionSensorUserFunction` and their three
siblings - by the machinery of RG12.4.4, with no further work.

**Gates**: 11/11 checks, the wheel, the full suite, the strict HTML build.

Also in this commit: the maintainer's `printToConsole eis set` -> `is set` reaching
`docs/generated/cInterface/Exudyn.md`, `python/exudyn/__init__.pyi`,
`src/Autogenerated/pybind_manual_classes.h` and `tools/generators/generated/stubAutoBindings.pyi`,
which is what a regeneration and a build do with a fix in `definitions/pybindModule.py`.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.19 - the arguments of a documented function, one per line (2026-09-26, #2665)

`utilityDocsModel.Tags2Markdown` ended every tag with `.replace('\n', ' ')`, which is right for a
description and wrong for a list: the `Args:` block of a docstring became one paragraph, and
`graphics.Sphere` - ten arguments - was eleven lines of prose in one. The names were in the body
font, so nothing marked where one argument ended and the next began.

The `input` tag is now written as a nested list, one argument per line, the name as code:

```markdown
- **input**:
  - `point`: center of sphere (3D list or np.array)
  - `radius`: positive value
```

`ArgumentEntries` reads the tag and **returns None unless the first line is an argument**, so a tag
that is prose is written exactly as it always was rather than guessed at; a description that goes on
over the following lines is appended to its argument. The only risk was a prose word taken for an
argument name, so the result was counted: **723 distinct names** over the utility pages and
`MainSystem.md`, of which the only one that reads like prose is `args`, which really is an argument
of `processing.py`.

**30 pages, 1865 lines.** The MainSystem extensions of the Python-C++ command interface are the same
code path and came with them - `mbs.CreateMassPoint` now lists its thirteen arguments - and the PDF
follows, because it is built from this Markdown.

**Gates**: 11/11 checks, the wheel, the full suite, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.5.2 - the connectors, and two signatures that were wrong (2026-09-26, #2664)

Eight connector items, ten user functions: 16 of 23 are defs now. The pages change in the header
spacing, in a blank line here and there - and in two places where they were **wrong**, which is the
part of this step that matters:

- `ObjectConnectorCoordinateSpringDamper.springForceUserFunction` advertised
  `..., offset, dryFriction, dryFrictionProportionalZone)`. Those two parameters were **removed on
  2023-01-21**, in V1.5.76: the item's own description says so, the argument table below it lists
  eight arguments, and the C++ `std::function` takes eight. Only the signature line still said ten,
  and it had said it for three and a half years. A reader who followed it wrote a user function that
  is never called with what it declares.
- `ObjectConnectorCoordinateSpringDamperExt.springForceUserFunction` has fourteen arguments and the
  signature was written over two source lines, so the page broke it mid-signature, in code font,
  after `velocityOffset,`. It is one line now.

Neither was found by reading: the first stopped the conversion because the table does not describe
`dryFriction`, the second because the signature did not parse. **That is what a generated block is
for** - a signature that is prose can disagree with the argument table beside it, and nothing notices.

**Three findings about the conversion itself**, each fixed in the script and each affecting the pages:

- a comment in `definitions/` that **holds content** is not a separator.
  `ObjectConnectorCoordinateVector` has a commented-out equation above its user function block, and
  the first cut walked through its `-->` and left the `<!--` open, which put a LaTeX equation on the
  page. The rule is now: a comment is walked over only when the whole of it is blank or `+++`.
- a comment line **inside** the prose is a paragraph break. `<!-- -->` renders as a blank line, and
  the writers of `ObjectConnectorCoordinate` used it as one; dropping it welded four paragraphs into
  two. `SensorUserFunction`, converted in RG12.4.5.1, lost a break the same way and gets it back
  here.
- a signature written over two lines is one signature.

`userFunctionExample` joined `CODE_KEYWORDS` in `tools/checkDefinitions.py`: the example of
`ObjectConnectorCoordinateSpringDamperExt` has a `#### ` comment in its Python, and a check for
heading levels read it as a heading. An example is code, like `miniExample` beside it.

**Gates**: 11/11 checks, the wheel, the full suite, the strict HTML build. The checks run **after** the
build when the interface has changed - `checkPython --stubs` compares the stubs with the *imported*
module, so running it against a wheel that is one step behind is a false failure, which is what it
reported here before the rebuild.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.4.5.3, .5.4 and RG12.4.6 - all 34 user functions, the gate, and what became redundant (2026-09-26, #2664)

**Every user function of every item is a Python def.** 34 of them; the 35th block is commented out in
`ObjectGenericODE1` and has no parameter, so it was left commented out. `tools/checkDefinitions.py`
now reports a parameter of a `PyFunction...` type that carries no def, and reports **none** today.

**Four more things the pages said that were not true**, all found by the conversion refusing to
believe a signature:

| item | what it said | what it is |
|---|---|---|
| `ObjectConnectorRigidBodySpringDamper.postNewtonStepUserFunction` | `(mbs, t, Index itemIndex, ...)` | C++ in a Python signature; the table below it says `itemNumber` |
| the same function's table | four rows and `\| ... \| ... \| other arguements see springForceTorqueUserFunction \|` | it has **thirteen** arguments, and the page now names all of them |
| `stiffness`, `damping` in **both** of its tables | `Vector6D` | the C++ takes `StdMatrix6D`, and the item's own example passes `np.diag([...])` - a 6x6 matrix |
| `ObjectJointGeneric.offsetUserFunction` and `_t` | `offsetUserFunctionParameters` and the return as `Real` | `StdVector6D`; the prose one line above says "offset vector for all relative translational and rotational joint coordinates" |
| `ObjectRigidBody2D.graphicsDataUserFunction` | `itemNumber` as `int` | `Index`, as on every other page |

Two blocks moved, and nothing else did: `ObjectRigidBody`'s note about `CreateRigidBody` is about the
**item**, so it stays in the description and the generated block follows it; and the example of
`ObjectConnectorRigidBodySpringDamper` sets `springForceTorqueUserFunction`, so it belongs to that
user function and now stands with it, above the `postNewtonStep` block instead of below it.

**Three more rules the converter needed**, each of which had put something wrong on a page before the
diff caught it:

- a signature **inside a comment** is commented out. `ObjectGenericODE1` has a whole block in one,
  with the `\startTable` LaTeX of before RG3.14 still in it.
- a comment over several lines is a **separator** only when it holds nothing but blanks and `+++`.
  One that holds text ends the block, and the text stays in the description.
- prose **after** the table is about the item, not about the user function, and stays.

**RG12.4.6 - what became redundant.** The argument table is now an output for all 34; the `\_`
escapes went with RG3.14; and the third question - whether `advancedUtilities`' hand-built `F(...)`
string can be replaced by the `Protocol` - is answered: it is not replaced, it is **joined**.
`userFunctionArgsDict` carries the Protocol's name as a fourth entry, and the error a user gets when
their function has the wrong number of arguments now ends with

    an editor checks a user function against exudyn.itemInterface.ObjectGenericODE2ForceUserFunction

so the message points at the thing that would have prevented it. Replacing the string entirely would
cost a user the signature in the message, which is what they are looking at when they read it.

**Gates**: 11/11 checks, the wheel, the full suite, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG4.1.1 - macOS is treated like Linux (2026-09-26, #2379)

The maintainer ran the suite on macOS ARM (V1.12.68.dev1, Python 3.13) and it exited non-zero on
**fifteen** models. Fourteen are test models and one is a **mini example**, which nothing had allowed
for: the exit code subtracted only the test models that a platform list names.

`UnresolvedOnMacOS()` stands beside `UnresolvedOnLinux()` in `runTestSuiteRefSol.py` and is the union
of it with the five models that differ only there, plus the mini example. The runner applies the list
on darwin, pytest judges by the same rule, and the mini example loop now counts a failure that a
platform list names separately, so it does not set the exit code either.

Checked against the maintainer's log rather than assumed: **all fifteen failures are covered**, and
one entry - `createSphereTriangleContact.py`, from the Linux list - passes on macOS, which is what a
"a difference here proves nothing" list should do. Windows is unchanged: the full suite still reports
`PASSED: no reproducible test failed`, and pytest 404 passed / 2 skipped.

The measurement is in the plan under RG4.1, with the relative error of each and which of them also
differ on Linux. Nine of fifteen do. **Nothing is four orders of magnitude out** - the largest is
1.4e-04 on a friction model - so macOS shows the same unexplained platform arithmetic as Linux on a
few more models, and no new category. The five macOS-only ones are RG4.1.2.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG11.3 - the results monitor beside a running simulation (2026-09-26, #2670)

`StartResultsMonitor(fileName, ...)` starts `python -m exudyn monitor` in a **second process** and
returns at once, so a script can watch its own results while it computes them:

```python
StartResultsMonitor('solution/sensorPos.txt', updatePeriod=0.5)
mbs.SolveDynamic(simulationSettings)
```

RG11.1 evaluated four ways and recommended this one; building it took the 60 lines it predicted,
because the file is the protocol the two processes already shared and the command line already
existed. Nothing is shared in memory, so there is no plotting inside the solver, no GIL question and
no backend question.

**The two questions RG11.1 left open are answered.** The child is **left running** when the script
ends - the point of a monitor on a short simulation is that the plot is still there afterwards - and
the returned `subprocess.Popen` is the handle for a script that wants it gone. `SolutionViewer` is
**not** served by the same call: it needs the renderer and the system in memory, not a file, so a
second process has nothing to give it.

**It respects the suppression flag**, which the maintainer asked for: with
`EXUDYN_SUPPRESS_UI_WINDOW_OPEN` or `suppressPlots` it starts nothing, prints why, and returns None.
A test that runs a script which calls it therefore neither opens a window nor leaves a process
behind - and the two examples below run in the example suite.

**Verified as a real subprocess**, not only by reading: started on a committed
`coordinatesSolution.txt` with `MPLBACKEND=Agg` and `--once --save`, the child exited 0 and wrote a
33 KB figure. The suppressed path was checked separately and returns None.

**Two examples use it**, one per kind of file: `springDamperTutorial.py` watches its **sensor** file
`solution/groundForce.txt`, and `3SpringsDistance.py` the **coordinates solution** that its long
integration writes.

`springDamperTutorial.py` also **crashed at its last line** and had for some time: it reads its own
output with `OutputFilePath(...)` and never imported it (#2669). It is not in the example suite,
which is why nothing noticed. One import line; found by running the tutorial to the end, which is
what adding the monitor to it required.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.8 - one test for all user functions at once (2026-09-26, #2671)

The maintainer, while RG12.4 was being finished: *"ideally there would be a test for all user
functions at once"*. `python/testing/test_userFunctions.py` is it - **41 test cases, 0.1 seconds**,
and not one of them runs a simulation.

That is the point of it. A model per user function is what `python/TestModels/` is for, and it tests
the solver; what had no test at all was whether the **four generated things still agree in the
installed package**: the entry of `userFunctionArgsDict`, the `Protocol` class, the item class that
takes the function, and the types the C++ interface exchanges. The generators compare them while they
run - and a stale generated file, or one hand-edited, is exactly the case a generator cannot see.

What it asserts, over all 34 (item, user function) pairs:

- every entry names a `Protocol` that the module really has, and it is exported in `__all__`;
- the `Protocol`'s `__call__` takes the argument **names** of the registry, in order, and there are
  as many types as arguments;
- no argument is still called `arg0` - the registry filled those in before RG12.4, and a leftover
  would mean a user function whose def was not read;
- the first argument is `mbs` of type `MainSystem`, which is what the C++ always passes;
- every `Protocol` carries a docstring, because that is what an editor shows;
- an item class **accepts an ordinary function** where its user function is and keeps it, which is
  the thing an annotation could break by turning into a check;
- every type in the registry is one of the fourteen the interface exchanges, so a new one cannot
  arrive unnoticed.

Checked that it bites rather than passes vacuously: renaming one argument in the registry at runtime
fails `test_protocolAndRegistryAgreeOnTheArguments` with both spellings in the message.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### The tracker and the plan say what is true (2026-09-26)

The maintainer: *"check whether the Still open list at the end of the plan is still up-to-date and
also shortly check the list of open issues since 2583, if there is anything already closed"*. Of the
nineteen issues open from #2583 on, **eight were already done** by work that is committed. Each was
checked before it was closed, not assumed:

| issue | what closed it | how it was checked |
|---|---|---|
| **#2664** a user function is described in five places and typed in none | RG12.4, all six sub-steps | `userFunctionModel.py` reads **34** user functions; `checkDefinitions` finds no parameter without a def |
| **#2669** `springDamperTutorial.py` crashes at the end | the import, in RG11.3 | the tutorial runs to its last line and prints the displacement |
| **#2658** `exudev docs --pdf` needs Perl | xelatex and makeindex directly | built with every `perl` directory off PATH; confirmed by the maintainer on their machine |
| **#2657** the visualization documentation has no structure | RG3.15 | `index.md` is the table of contents the maintainer decided |
| **#2661** the command line and the monitor are chapters of their own | RG3.15 | both are nested under *Advanced topics* (`introductionAdvanced.md:418`) |
| **#2662** the manual mixes sentence case and Title Case | RG3.16 and `checkHeadings` | 353 headings pass the gate |
| **#2652** the item docstrings show `addExampleImage{X}` | RG3.14's converter | `help(ObjectJointRevoluteZ)` ends with the marker types; the string is gone from the package |
| **#2497** 59 bare `except:` in the shipped package | the module clean-ups, one at a time | **zero** in `python/exudyn/` and **zero** `E722` in `tools/ci/ruffBaseline.txt`, where all 59 were listed |

**#2497 is the one worth a sentence.** It was raised in 2021 as #1988, re-raised as #2497, and
baselined in revision2026 step R5.5.3 rather than fixed, *"because each needs a decision on which
exception was actually meant"*. Nobody ever sat down to do it; it went out with the module
clean-ups, one `except Exception as e` at a time, and the baseline emptied without anyone noticing.
The check that made a new one fail is what kept the number from growing back.

**Closing a batch has an order, which this taught.** The version an issue carries is derived from
its position among the closed issues **sorted by issue number**, so eight `resolve` calls in the
order they were thought of made `checkIssues` report that six published version numbers had moved.
Nothing was wrong with the issues; the numbers simply have to be handed out in ascending order. The
repair was to put the eight files back and resolve them again, 2497 first and 2669 last - and the
rule for the next batch is: **resolve in ascending issue number**.

**The plan was corrected in the same pass.** *Still open* lost the rows of what is done and gained
what was missing, and is sorted by step number again - 18 of 29 rows had drifted out of order.
*Raised by the current work, and not yet a step* is **empty but for one row**: the maintainer asked
for its entries to become steps, so they are **RG3.21** (#2673, the pages that describe the state
before a step that is done - whose first job is the list), **RG3.22** (#2659), **RG4.6** (#2674, a
test hook for `forceQuitSimulation`: #2616 fixed the behaviour and nothing can reach it),
**RG4.7** (#2423) and **RG10.2** (#2541). What stays in the table is #2608, because the maintainer
asked for a *suggestion* and not a step: that is **RG6.2.26**, and it waits for RG12.5.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.14.12 - the theDoc.pdf references (2026-09-26, #2656)

Five hand-written docstrings sent a reader to a document that has not existed since decision D8.
**The step was smaller than it looked, and measuring said so before any work was done**: both targets
the two `Returns:` lines carry - `sec:solverSubstructures` and `sec:MainSolverStatic` - **already
resolve**, because `(sec-solversubstructures)=` is in `CSolverStructures.md` and
`(sec-mainsolverstatic)=` in `MainSolver.md`. Only the words *check theDoc.pdf* in front of them were
wrong. So five edits, not five judgements:

| where | now |
|---|---|
| `__init__.py:11` | the header points at `https://exudyn.readthedocs.io/`, as `README.rst` does |
| `solver.py`, `SolveStatic` | *"see MainSolverStatic, [Section](#sec:MainSolverStatic), for further details"* |
| `solver.py`, `SolveDynamic` | the same with `MainSolverImplicitSecondOrder`, which the old text named without linking |
| both `Returns:` lines | *"see ... and the items described in ..."* - the two links, no PDF |
| `mainSystemExtensions.py:3369` | *"as compared to the reference manual"* |

**The class name stays beside the link, and that is not decoration.** A `[Section](#x)` link is
rendered by `docstringText` as the bare anchor - `see sec-mainsolverstatic for further details` -
because a stub has no page to link to. Naming the class in the prose gives the HTML a link and the
tooltip a word a reader can search for.

**And the same two docstrings were wrong about their own solver.** `SolveDynamic` said, twice, that
`storeSolver` stores *the staticSolver object ... as mbs.sys['staticSolver']* - it stores
`mbs.sys['dynamicSolver']` (`solver.py:273`). A copy of the static docstring that nobody re-read.
Fixed in the same lines, because they were being rewritten anyway and leaving it would have meant
writing a correct sentence around a wrong one.

The name itself is said **once**, in `docs/manual/revisions.md`, where what changed between releases
belongs: until this release the documentation was one PDF called `theDoc.pdf`, a name that still
appears in older issues and notes, and it is the HTML documentation now.

**Gates**: 11/11 checks, the wheel, the full suite, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.8.4 - the figures the conversion lost are back (2026-09-26, #2594)

Three of the four. Each was found in the 1.11 sources the maintainer kept in `tmp/docs/`, with the
caption it had there:

| figure | where it is now | what it shows |
|---|---|---|
| `generalContactSpheres` | `theoryContact.md`, *Sphere-sphere contact*, after the contact point equation - where `theory.tex` had it | the geometry of two spheres and their markers |
| `generalContactANCF2Dcircle` | the same file, *Contact relations for ANCF cable* | the two cases of a cable span intersecting a circle |
| `ObjectJointALEmoving2D` | the description of `ObjectJointALEMoving2D`, after the velocity difference | the geometry of the ALE sliding joint |

**One sentence was waiting for its figure.** The ANCF paragraph read *"... intersects with the
circle, see the geometrical relations between the beam span and the circle"* - the conversion had
replaced `\\fig{fig_generalContactANCF2Dcircle}` with a description of the figure it was dropping.
It says `see {ref}`fig-contact-ancf2dcircle`` now, and there is something to see.

**The ALE figure had been hidden on purpose**: in `itemDefinition.tex` it sat inside
`\\ignoreRST{...}`, so the conversion that produced the web documentation honoured the instruction
and dropped it. That was a decision for a format that no longer exists.

All three are `.*` candidates - the pair mechanism of RG3.8.1 - so the browser shows the SVG the
maintainer drew and the PDF the vector original. **Checked in both builds**, not only in the HTML:
`_buildpdf/latex/exudynDocumentation.tex` includes `{generalContactSpheres}.pdf`,
`{generalContactANCF2Dcircle}.pdf` and `{ObjectJointALEmoving2D}.pdf`, and the PDF built in 2 passes.

**What is left of #2594 is RG3.8.5**, and it is left deliberately. Seventeen figures exist as a
`.png` **and** as a `.pdf` or `.eps`, and every reference names the `.png`; writing `.*` would give
the PDF the vector original. But the two were exported at different times over ten years, and if a
pair has drifted the HTML shows one picture and the PDF another **with no warning at all**. One
comparison per pair first; the `.png` in both builds is at least the same thing twice. `intro2.jpg`
stays where it is: unreferenced, and deleting a tracked file is the maintainer's word.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.18 - a page does not repeat its own title (2026-09-26, #2660)

Six pages of the Python-C++ interface and **178 item pages** opened with their own name twice.

**The two halves of the issue were one line of code.** `latexToMarkdown.DropRepeatedTitle(text,
title)` removes a first heading that only repeats the page title and **returns its label**; the two
emitters write that label above the `# ` title instead. Nothing lifts the headings that follow:
`NormalizeHeadings` already rebuilds every level from the nesting it walks, so a heading that is
gone takes its level with it. That is what the table of contents needed -
*MainSystem extensions (create)* and *(general)* were at level 3 **because** the repeated section
held level 2, and both toctrees say `:maxdepth: 3`.

Measured in the built HTML, which is the only proof that counts here:

```html
<li class="toctree-l3"><a href="...MainSystem.html#mainsystem-extensions-create">MainSystem extensions (create)</a></li>
<li class="toctree-l3"><a href="...MainSystem.html#mainsystem-extensions-general">MainSystem extensions (general)</a></li>
```

Both entries are in the table of contents for the first time.

**The item pages were the delicate half** and needed no special handling in the end: the second
heading carried `(sec-item-objectground)=`, which every item reference in the documentation points
at, and the label simply moved above the page title. A target on a heading resolves from another
page - the property RG3.14.7 established - and the strict build is what proves it, together with a
**PDF build**, because a moved label is what the LaTeX writer fails on when the HTML does not.

The six interface pages carried **no label at all** on their repeated section, which was measured
before anything was moved: nothing referenced them, so there was nothing to preserve.

105 generated pages changed, 419 lines added and 629 removed - one heading and one blank line each,
and every `### DESCRIPTION of X` is now `##`.

**Gates**: 11/11 checks, the full suite, the strict HTML build and the PDF.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG4.2 - a computed value that Python reads (2026-09-26, #2413)

The step had said since revision2026 step R10.2 that `pContact` should *become a data variable*. The
maintainer decided otherwise: **make it what the FFRF members are** - computed inside the core, read
from Python, not written.

**It already was**, and measuring said so before anything was changed:
`MainObjectContactConvexRoll.h` answers `GetObjectParameter(..., 'pContact')` and puts it in the
dictionary, and there is **no** branch for it in `SetParameter`. So the work was to say it - the
description said *"The  current potential contact point. Contact occures if pContact[2] < 0. "*, with
a double space, a typo and no hint that a user may not set it.

**What the measurement did find is `rBoundingSphere`, one parameter above it.** Also computed - from
`coefficientsHull`, in `CObjectContactConvexRoll::InitializeObject` - and it **was settable**. Writing
it did nothing at all: `SetObjectParameter` ends in `ParametersHaveChanged()`, which recomputes the
value that was just written. A user could set it, read back something else, and never be told. It is
`CFReadOnly` now, so the attempt raises, and the generated interface lost its setter and its
dictionary write.

**Both are tested rather than asserted.** `python/testing/test_computedParameters.py` builds a roll
on a ground and checks the four things that matter: `pContact` is a finite 3D point,
`rBoundingSphere` **is** the hull polynomial at 0, both appear in `mbs.GetObject(...)`, and writing
either of them raises.

**One reference value moved, and it is the right one.** `parameterConversionTest.py` probes every
parameter of every item along four paths, and the four write paths of `rBoundingSphere` - `class`,
`dict`, `set`, `omit` - are gone. Its reference file records outcomes per parameter, so it was
rewritten with `recordReference = True`; the model's result is back to 0 and the diff is the two
lines that name `rBoundingSphere`. Nothing else in 4625 parameter paths changed, which is the
evidence that the read-only flag touched what it was meant to and nothing else.

**Gates**: 11/11 checks, the wheel, the full suite, pytest 450 passed / 2 skipped, the strict HTML
build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.5.1 - one file for the settings that persist (2026-09-26, #2666)

`~/.exudyn/config.json`, one file, two sections read today:

```json
{"config": {"outputDirectory": "solution/"},
 "visualizationSettings": {"openGL.multiSampling": 4, "nodes.basisSize": 0.5}}
```

**Three of the four open questions are answered, and the fourth is split off.**

- **What may be overridden**: a plain value - a number, a flag, a string - or a list of them.
  Anything else is **refused with a message**: a setting holding graphics data, a user function or a
  matrix container cannot be carried honestly by a JSON file, and guessing is how a settings file
  starts corrupting models. A key that names no setting is reported the same way, and neither stops
  the import.
- **Who reads it**: `python/exudyn/__init__.py`, beside the environment variables it already reads,
  through `python/exudyn/settings.py`. The C++ core is untouched - no `py::module_::import("json")`
  in the core and none of its failure modes - and the values are in place before a script can look
  at them.
- **What a script can ask**: `exudyn.settings.Applied()`, `Ignored()` and `Print()`. That is the
  answer to *why does this behave differently here*, and it is printed as **one note** at import
  naming how many settings came from the file.

**The `visualizationSettings` half needed a decision that the step had not seen**: they do not exist
until a `SystemContainer` does. They are applied **when one is created**, through a Python subclass
of `SystemContainer` that `__init__.py` installs - and installs **only when the file holds
visualizationSettings**. With no file, or a file without that section, `exudyn.SystemContainer` is
exactly the class the compiled module defines, so the normal case carries none of this at all.
Checked that the subclass is a full SystemContainer: `AddSystem`, `Assemble` and a node all work
through it, and `mbs.GetSystemContainer()` returns the same C++ object whose settings were already
applied.

**A stored setting must never move a test result**, and that is the part with teeth.
`EXUDYN_NO_USER_SETTINGS=1` ignores the file, and `runTestSuite.py`, `runTestExamples.py`,
`runPerformanceTests.py` and a new `python/testing/conftest.py` set it **before exudyn is
imported** - a child process inherits it, so the workers of `--parallel` and of pytest are covered.
Without this, a maintainer who stored an output directory would have moved results on their machine
and nowhere else.

**11 tests** in `python/testing/test_userSettings.py`, none of which touches the real file:
`EXUDYN_CONFIG_FILE` names one in the pytest temporary directory. They check that a stored setting
arrives, that a typo is reported and changes nothing, that a non-plain value is refused, that a
visualization path is applied by its dialog path, that an unknown section and a broken file are
reported and survive, that `Store` writes what differs from the defaults and nothing else, and that
the runners ignore the file.

**Nothing writes the file by itself.** `exudyn.settings.Store(SC)` writes the settings that differ
from the defaults - the same list the dialog marks as changed - and `Clear()` deletes it. A script
that behaves differently on another machine because something was stored there is the failure this
file has to be worth, so storing is always asked for.

Documented in `docs/manual/userSettings.md`, under *Tools that are not part of a model*, and named
in the revisions chapter, because it changes what a script does when the file is present.

**Gates**: 11/11 checks, the wheel, the full suite, pytest 461 passed / 2 skipped, the strict HTML
build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.5.3 (= RG6.2.29) - the dialogs remember where they were (2026-09-26, #2675)

Undecided since 2026-09-23, proposed and approved on 2026-09-26, and built into the file RG12.5.1
made three hours earlier - which is the argument the proposal made: a second file for window states
would have been the beginning of one file per feature.

```json
{"dialogs": {"visualizationsettings": {"size": [1024, 768], "position": [100, 80]}}}
```

**The rule RG6.2.11 wrote down is now code**, in `settings.PositionIsReachable`: the **size** comes
back always, the **position** only when the window would still be reachable on the screen it would
appear on. A monitor unplugged, a laptop undocked, a resolution changed - each of them would put a
dialog where nobody can reach its title bar, and a settings dialog that cannot be closed is a stuck
session. When the position is refused the dialog opens where it would have opened anyway, at its
remembered size.

**It is switched on, not default**: `visualizationSettings.dialogs.storeDialogPositions`, new and
False, which is why the note of RG12.5.1 covers it - a remembered window is exactly the kind of
state that makes a bug report irreproducible.

**Three things that only show up when you build it, not when you propose it:**

- the geometry cannot be read after the window is destroyed, and a dialog is left in three ways -
  the close button, Escape, the window manager. So it is recorded on every `<Configure>` and the
  last value is the one stored.
- a window manager reports a screen left of the primary one as `900x700+-1500+40` or as
  `900x700-1500+40`, depending on which one it is. Both are parsed, `nonsense` and `''` are
  refused, and all four are a test.
- `GetRendererSystemContainer()` is None for a dialog of `python -m exudyn dialogs`, which has no
  SystemContainer to ask - so nothing is stored there rather than crashing.

**Tested without opening a window**, which rule 11 requires and which is possible because the two
halves are separable: `PositionIsReachable` is a pure function of a position and a screen rectangle
(five cases, including a virtual desktop that starts at a negative x), and the file layer is tested
through `EXUDYN_CONFIG_FILE`. What is not tested is the window itself; that is for a human with a
screen.

**One reference value moved**: `parameterConversionTest.py` probes every parameter of every item and
`storeDialogPositions` is a new one, so its two paths are in the reference now. 4627 parameter paths,
two of them new.

**#2608 was already closed**, and resolving it again was the second time in two days that an old
issue was re-resolved: RG6.2.11 closed it on 2026-09-23 with the window question left undecided, so
the leftover needed an issue of its own (#2675) rather than a second stamp on a closed one. The
symptom is the same both times - `checkIssues` reports that a published version number moved,
because the check sorts the closed issues by `dateResolved` and a re-stamped date moves an issue
from its old place into today's block. The repair is the same too: put the file back and raise the
issue the work actually needs.

**Gates**: 11/11 checks, the wheel, the full suite, pytest 476 passed / 2 skipped, the strict HTML
build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.6 and RG12.7 - the columns and the font of a dialog (2026-09-26, #2667, #2668)

Two small things the maintainer asked for on 2026-09-26, and one bug they found.

**The columns are fractions of the dialog** (#2667). `columnWidthName`, `columnWidthValue` and
`columnWidthType` in `visualizationSettings.dialogs`, each a share of the width, and the description
column takes what they leave - which is what makes three numbers enough. The defaults - 0.31, 0.18
and 0.11 - give 317, 184 and 112 pixels at the default width, where the hard-coded numbers gave 325,
188 and 113: the old ones were a sum of 1046 in a dialog of 1024, so they were fractions all along,
of something that did not exist.

`ColumnWidthFractions` applies the rule that three independent settings need: each is at least 0.05,
and if together they would leave the description less than a tenth of the dialog, all three are
scaled down to leave it that much. A user sets three numbers in any order; refusing the third one
because of the first two would be the wrong half to complain about.

**Ctrl and the wheel change the font size** (#2668), about 10% per notch, between 0.4 and 4 times
the scaled size - a dialog whose font is two pixels tall cannot be read back to a usable size. The
row height, the column widths and the font of a changed row all follow it, because every one of them
is computed from the font factor. `Ctrl` and not the bare wheel, which scrolls the tree; on X11 the
same is `Control-Button-4/5`, which is bound as well.

**Tested in a withdrawn root**, the pattern `test_guiValues.py` already used: the widgets are real -
a whole settings tree - and no window is ever mapped, which rule 11 requires. Seven tests: the
fractions, their normalisation, the floor, the four columns of a built tree, the font going up and
back, and the two clamps.

**What the tests found is worth more than the two features.** They failed at random in a parallel
pytest run - `RuntimeError: Access violation - no RTTI data!` in `DialogScaling`, reading
`guiSC.visualizationSettings.dialogs.fontScaling`. `GetRendererSystemContainer` had been made to
survive a destroyed SystemContainer in #2623, **but only the lookup**: it checks that the entry
exists and has the right type, and the first *use* of what it returns raises in whichever caller
comes next. One cheap read inside the guard turns that into the `None` every caller already handles
(#2676). A user meets it when a dialog is opened after the container that started the renderer has
been deleted; the tests met it because a worker runs many models in one process.

Six consecutive full parallel runs are clean afterwards. If it ever comes back, this is where to
look.

**Gates**: 11/11 checks, the wheel, the full suite, pytest 482 passed / 2 skipped, the strict HTML
build. `parameterConversionTest.py` gained the three new settings - six parameter paths - and its
reference was rewritten.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.14.13 - the LaTeX machinery of autoGenerateHelper.py (2026-09-26, #2655)

**Audited by reachability, not by reading.** Every top-level name of the module, every name any other
file in `tools/` mentions, and a walk from the second set through the first: what the walk does not
reach is dead, whatever it looks like. Eight names, **185 lines**:

| what | why it was there |
|---|---|
| `convLatexWords` (65 lines), `convLatexCommands` (63), `convLatexMath` (14), `convLabelEq` | the conversion tables of the LaTeX-to-RST machinery |
| `ReplaceWords`, `FindMatchingBracket` | the two functions that used them |
| `abc`, and the loop that indexed it | filled `convLatexMath` with `\av`, `\Am` and their 50 siblings |
| `ArgNotSet` | a sentinel nothing sets |

**The proof is that the regeneration is a no-op**: `exudev generate` reports *"regeneration is a
no-op; the committed generated set is current"*, so 185 lines left the generator stack and not one
byte of its output moved.

One thing the walk missed and the build caught at once: a module-level **`for` loop** that filled
`convLatexMath` from `abc`. A reachability walk over definitions and assignments does not see a bare
statement, and the generator failed with `NameError: name 'abc' is not defined` on the next run.
Which is the argument for the gate rather than for a cleverer walk.

**`PyLatexRST` took two arguments it ignored** - `sLatex` and `sRST` - *"because the declarations
pass them positionally"*. Three declarations in `pybindEmitter.py` passed `('','', '')`; they pass
nothing now and the two parameters are gone. The **name** is the last LaTeX in the file: the class
writes Python, a stub and Markdown, and has written neither LaTeX nor RST since RG3.14. Renaming it
touches five emitters, so a note stands where the class is defined and it waits for the next change
to them.

**What is alive, and why** - the audit is not only what was removed:

| name | uses | what it really does |
|---|---|---|
| `Str2Latex` | 21 | two jobs: a C++ default value in Python spelling (`true` to `True`, `EXUstd::InvalidIndex` to `invalid (-1)`), and escaping `_`. The first is needed everywhere; the second is RG3.14.14 |
| `Str2Doxygen` | 15 | the C++ side: a Doxygen comment, where the escaping is correct |
| `GetTypesStringLatex` | 7 | the requested marker and node types of an item page |
| `Latex2RSTlabel` | 1 | one label in `utilityDocsEmitter` |

**One thing from RG12.7 landed here**, because it was found while these gates were being run: the
three tests that build a settings tree were **flaky under `pytest -n 8`**, one run in three, as a
hard worker crash - *"node down: Not properly terminated"*, no Python traceback. Two attempts and a
measurement:

- rewriting the clamp test from 180 font changes to two changed nothing, so it is not the number of
  calls;
- **one Tk root per process** is a real fix for a different problem: a second `tk.Tk()` after the
  first was destroyed fails on this Windows build, which is why three tests reported *"no tkinter
  display available"* although the display was there. With a cached root the serial run went from
  three skips to **73 passed**;
- the parallel crash remained, so the three run **serially only**, with the measurement in the skip
  reason. A crash reported as a failing test costs an hour of somebody's day.

**And the audit found what the step could not know.** 33 generated pages carry **745** backslash
underscores. 707 of them are outside mathematics, where Markdown renders the escape as a plain
underscore - the page looks right and the source carries LaTeX nothing needs. **38 are inside
mathematics and are a defect**: there `\_` is a literal underscore, so `BeamSectionGeometry` shows
`c_Y` as text where a subscript was meant, and a display equation of `ObjectJointGeneric` carries
`UF\_t_{k}`. That is **RG3.14.14** (#2677), and it is why #2655 does not close today: each of the
38 needs a judgement, and the last hour of a long session is not when to make 38 of them.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.14.14 - the LaTeX escapes in the generated Markdown (2026-09-26, #2677)

**The first thing the step did was correct its own number.** 745 backslash-underscores were counted
by RG3.14.13; **638 of them are the tracker log**, where `ToMarkdown` escapes the text of an issue on
purpose since #2545 - an author writes text, not markup, and a `|` would otherwise split a table
cell. Those are right and stay. The real count was **115**.

**Five producers, all of them escaping for a format that is gone:**

| where | what it escaped |
|---|---|
| `autoGenerateHelper.Str2Latex` | every `_` of every string it was given |
| `structureDocsEmitter` | the whole description of a settings parameter, mathematics included |
| `itemDocsEmitter` | the name of an output variable: `Coordinates\_t` |
| `itemModel.ExtractLatexSymbol` | the text beside a symbol |
| five example files | a hand-written `#**output:` comment |

**38 of them were a real defect, and it is the reason this was a step.** Inside `$...$` a
backslash-underscore is a *literal underscore*, so the subscript the author wrote never appeared:
`BeamSectionGeometry` showed `c_Y` as text where `$c_Y$` was meant, and `SimulationSettings` had
**23** of them - `$h\_{max}$`, `$t\_{end}$`, `$a\_{tol}$`, `$q^{Ref}\_i$`. The sources were
correct all along; the emitters broke them on the way out. They render as mathematics now.

**One needed a judgement rather than a rule**: a display equation of `ObjectJointGeneric` carried
`UF\_t_{k}`, which is not even valid LaTeX (a double subscript). Two equations above it, the same
user function is written `UF_{t;0,1,2}`, so the velocity-level one is `UF_{t;k}` - the author's own
notation, not an invention of this step.

**The five example comments became code spans**, not bare names: `initialValues_t` is a name, and
`[_t]` in running text opens emphasis in Markdown, which is what the escape had been avoiding. A
name belongs in backticks; the escape was the wrong answer to a real question.

**Two functions got their examples back.** `AngularVelocity2EulerParameters_t` and
`AngularVelocity2RotXYZ_t` had **no** *Relevant Examples* list, because the keyword search looked
for `AngularVelocity2RotXYZ\_t` and no example contains a backslash. Nobody would have found that by
reading; it fell out of removing the escape.

**And the full PDF build found a reference I had broken three steps earlier**: the description of
`storeDialogPositions` (RG12.5.3) says `[](#sec:usersettings)` where the label is
`sec-usersettings`. The incremental HTML build never re-read that page, so it passed; the fresh
build for the PDF reported it at once. The colon spelling is the LaTeX one - which is the same
mistake this step is about, made by hand.

**Gates**: 11/11 checks, the wheel, the full suite, pytest, the strict HTML build **and the PDF**,
which is what a change to mathematics has to pass.

**RG3.14 is finished** and #2655 closes with it: the descriptions in `definitions/` are Markdown,
the converter is what reads them, the rules are in `definitions/README.md`, the checks hold them,
and what is published carries no LaTeX escapes any more.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG6.2.28 - a blocker that was not there (2026-09-26, #2678 closed as obsolete)

The step was proposed on 2026-09-26 as **the** blocker of RG12.9 to RG12.11: the four lights and the
ten raytracer materials are initialised by the `SystemContainer` and not by the settings structure,
so the override settings would have no unambiguous default and no single moment of application. The
maintainer read the C++ and replied that it looked done already.

**It was, and the measurement is one line**: `exu.VisualizationSettings()` and
`SystemContainer().visualizationSettings` differ in **0 of 470** settings. `RG6.2.20` (#2626) did it
on **2026-09-23**, three days before the issue was raised: the 89 assignments moved out of the C++
constructor into `definitions/structureDefsVisualizationSettings.py`,
`containerInitialisedSettings` is empty, `DefaultSettingsDictionary` takes the plain constructor -
which is also what keeps the dialog from creating a container it would have to detach again - and
`testASystemContainerInitialisesNothingBeyondTheDefaults` requires the difference to stay empty.
Nothing needed changing.

**Where the wrong premise came from, because it is the part worth keeping.** The RG6.2.12 log entry
says, in the present tense, that a `SystemContainer` initialises 59 settings its constructor does
not. That was true on 2026-09-23 **when it was written**, and a closed log entry is never edited -
which is exactly right for a log and exactly wrong as a source for planning. The plan step repeated
it, and it would have put a C++ change in front of three steps that do not need one.

**A premise that decides the order of three steps is measured, not read.** It cost one command.

`#2678` is closed as obsolete rather than resolved: nothing in this session changed what it asks
for, and a resolved issue that changed nothing is a version number that means nothing.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.24.1, RG3.24.2 - what the legacy string helpers still do (2026-09-26, #2681, #2682 raised)

The maintainer's suspicion, in their words: *"I still believe that functions like Str2Latex and in
particular DefaultValue2Python would now be replaced by the new generators and definitions
mechanisms."* Two sub-steps: where they are called (**RG3.24.1**) and what they still do
(**RG3.24.2**). Nothing was changed - this is the measurement that RG3.24.3 will act on.

**Nine helpers, 60 call sites, all inside `tools/generators/`.** The interesting number is not the
total but where it is zero: `SplitString` and `CutLinesFromString` are **imported by
`itemDocsEmitter` and never called**.

**How they were measured.** `generate.py` runs each generator as a **subprocess**, so an in-process
wrapper around a helper sees nothing at all. Every call site was instead given its real inputs,
taken from the models the emitters read - 2837 item parameters, 883 settings parameters, 52 structure
descriptions - which is reproducible and does not touch the tree.

**`Str2Latex(s)` with its default arguments is a no-op, and now provably**: of **3720** type names,
sizes, python names and descriptions, it changes **0**. After RG3.14.14 took the underscore escaping
out, all that is left is `{` to `\{` - a LaTeX escape written into a **Markdown** page, and into a
**stub file** at `structureStubEmitter.py:56`, where it would be a syntax error if a python name ever
contained a brace. Five of the 21 call sites are of this kind and can go without a decision.

**`Str2Latex(s, isDefaultValue=True)` is not a LaTeX function at all.** It is a C++-to-Python
converter, and it is the **only** source of the printed default of **990** item and **263** settings
parameters. There are therefore **two** such converters in the same file, and they disagree:
`Matrix6D(6,6,0.)` becomes `np.zeros((6,6))` on a documentation page and
`IIDiagMatrix(rowsColumns=6,value=0.)` in `itemInterface.py`. Both are right for their reader, which
is why neither was ever noticed.

**`GetTypesStringLatex` writes `\texttt{...}`** - a LaTeX macro into a Markdown page - and **0** of
them reach `docs/generated/`, because both callers strip it again. A function whose output is
undone by every caller.

**And then the one worth the step, `DefaultValue2Python`.** A default value travels
**Python value → C++ literal string → Python value**: `itemModel.DefaultValueString()` calls
`definitionTypes.CppLiteral()`, and `DefaultValue2Python()` converts the result back with about
twenty `str.replace()` calls and three substring `find()` heuristics on `'Index'`, `'Float'` and
`'Vector'` - **322** values steered by a substring rather than by their type. 1001 of 2837 inputs are
changed by it.

**It ends with an unconditional `s.replace('f', '')`.** The intent is the float suffix of `0.05f`;
the effect is that the letter `f` is deleted from whatever else is there.
`Transformation66List()` becomes **`Transormation66List()`** for three members of
`ObjectKinematicTree`, and `NoDefaultValue` becomes `NoDeaultValue` for a settings parameter.

**Nothing visible is wrong today**, and that is the finding, not a reassurance: the three members are
`CFNoInterface`, so the mangled value never reaches `itemInterface.py`, and the settings default is
filtered before the call. The defect is contained **by accident**. One new parameter whose type name
contains an `f` and not the flag would ship a silently wrong Python default, and no gate would say
so. What does reach the tree is cosmetic and comes from the same cause: `coefficientsHull` of
`ObjectContactConvexRoll` has `defaultValue=' Vector()'` with a **stray leading space**, and the
generated signature reads `coefficientsHull =  []` - a default that is a *string* is never validated.

**Why it cannot simply be deleted, which is the part the suspicion did not cover.** Of the 872 item
members with a default, **470 are plain Python values** (234 float, 181 bool, 55 int) and make the
round trip for nothing. The other **402 exist only as C++ source text** - 142 `CppValue` and 260
plain `str`, in 29 distinct expression shapes - and for those `DefaultValue2Python` is not a
redundancy but the **only place where the Python default is defined**. The new mechanism has replaced
half of this function and not the other half; the options in RG3.24.3 are about which way the
remaining 402 go, and #2682 records it.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.9 - the override settings live in exudyn.special.overrideSettings (2026-09-26, #2679)

RG12.5.1 built the file and kept its values in a Python module. The maintainer, having seen it:
*"they anyway should be used rarely and with caution; they are for convenience ... location in
exudyn should be therefore e.g. `exudyn.special.overrideSettings`"*, and *"this requires a dict on
the C++ side (?) ... Advantage: also accessible from C++ then"*. Both are done.

**The decided carrier, and why it is not a member.** The step said "a `py::dict` member of
`PySpecial`". A member means `pybind11` in `Main/Experimental.h`, and that header is included by
**eight** translation units - `Linalg/LinearSolver.cpp` and `Utilities/Threading.cpp` among them,
which have no business knowing about Python. The dictionary is instead
`EPyUtils::OverrideSettings()`, declared in `PybindUtilities.h` and defined in
`Pybind_manual_classes.cpp`:

```cpp
py::dict& EPyUtils::OverrideSettings()
{
    static py::dict* overrideSettings = new py::dict();
    return *overrideSettings;
}
```

**It is never freed, and that is the point.** The step itself named the caveat - *"a global that owns
Python objects must not be destroyed after the interpreter"* - and it is why `exu.sys` is a module
attribute and not a C++ member. A function-local static pointer is allocated on the first access,
which happens during module import while the interpreter and the GIL are there, and has no
destructor to run afterwards. From Python it is exactly what was asked for,
`exu.special.overrideSettings`, exposed **read-only** so that it cannot be replaced by something
that is not a dictionary - `update`, `clear` and item assignment all work, which is everything that
is needed - and `repr(exu.special)` ends with `overrideSettings: n section(s)`.

**The module moved and was renamed**: `python/exudyn/settings.py` to
`python/exudyn/misc/overrideSettings.py`, *"it is not intended to be used by the user"*. A module of
its own rather than a merge into `misc/settingsUtilities.py`: that one is about **editing** a
settings structure, this one about **persisting** it, and they share no code.

**What changed beyond the move, and it is the part that fixes something.** Every function that took
`settings=None` read the file **again**: `ApplyConfig`, `ApplyVisualizationSettings` and -
worst - `DialogGeometry`, which is called whenever a dialog opens. A file edited during a run was
therefore read several times and the dialogs could disagree with the settings. `None` now means the
store, the file is opened once by the import, and `Store` and `StoreDialogGeometry` write **both**
the file and the store so that the two cannot drift apart inside a process. `Settings()` is the new
accessor and returns the dictionary itself, not a copy.

**The three new tests say what a reader needs**: that `Settings() is exu.special.overrideSettings`,
that the store is **empty** under the runners (a test that picks up a setting from the machine it
runs on is the failure this whole mechanism has to avoid), and that `settings=None` takes the store
and not the file. The fixture of the eleven older tests now fills the store as the import does; it
empties it again afterwards, because a dictionary on the C++ side outlives a test.

**Gates**: 11/11 checks, the wheel, the full suite, 29 tests in `test_userSettings.py` and 447
in `python/testing`, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.23 - the override settings are documented where the module is (2026-09-26, #2680)

The maintainer chose option A: *"integrated into the Python-C++ command interface ... under the
Exudyn module"*, so that there is **one** source. `docs/manual/userSettings.md` was written by hand
in RG12.5.1; its text is now a section of the generated Exudyn module page,
*Settings that persist between runs*, written as `pb.AddDocu(...)` in `definitions/pybindModule.py`
after the table of functions, with `AddDocuCodeBlock` for the JSON and the Python. The manual page
keeps its place in the table of contents - *Tools that are not part of a model* - as a pointer to it,
which is what the decision asked for; the label `sec-usersettings` stays on the pointer so that
nothing outside breaks, and the new `sec-overridesettings` is what the two references now point at.

`exu.special.overrideSettings` is documented as a data member beside `exu.sys` and `exu.variables`,
which is where a reader of the module page will look for it.

**The environment variables needed the list more than the settings needed the move.** The maintainer
asked for one *"where they essentially affect behavior"*, and asked whether there already was one:
there was not. The package reads **six**, and three of them - `EXUDYN_NO_USER_SETTINGS`,
`EXUDYN_CONFIG_FILE` and `EXUDYN_IMPORT_VERBOSE` - were documented **nowhere**;
`EXUDYN_OUTPUTDIRECTORY` and `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` were named only in passing in
`revisions.md` and `EXUDYN_MODULE` only in `commandLine.md`. They are now a table under
`sec-environmentvariables`, each with what it does and why anyone would set it. A seventh,
`EXUDYN_MACHINE_ID`, is read only by the repository's performance runner and names a log file, so it
is not in a user's list.

**The troubleshooting hint**, in the maintainer's words *"users experiencing weird behavior shall
delete the `~/.exudyn` folder"*, is a subsection of *Errors: what Exudyn raises, and what to do about
it*, called *Behaviour that is not in your script*. It says the stronger thing first: deleting the
folder returns everything to the defaults and nothing in it is needed to run a model, and
`EXUDYN_NO_USER_SETTINGS=1` answers the question **without** deleting anything.

**Two rules were learnt from the gate, not from the README, and one of them was the README's fault.**
Inline code in a description is a **backtick span**: `checkDefinitions` rejects `\texttt{...}`
outside mathematics, while `definitions/README.md` said *"Inline code ... is `\texttt{...}`"*. RG3.14
migrated the descriptions and the rule was not migrated with them, so the document that CLAUDE.md
rule 6b points at was telling a writer to do what the gate forbids. It now says what is checked; 42
`\texttt{}` were written and converted. The second: a sub-heading of an `AddDocu` section is level
**4**, because the section itself lands at 3 on the module page - the checker says which level it
wants, and it was right both times.

**Gates**: 11/11 checks, the wheel, the full suite, pytest, the strict HTML build **and the PDF**,
because a new section with two labels and a table is what the LaTeX writer fails on.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG3.24.3 - the default values stopped travelling through a C++ literal and back (2026-09-26, #2682)

RG3.24.1 and RG3.24.2 measured the round trip; this built option A. Three findings changed the shape
of the work, and the first two made it smaller.

**`CppValue` already knew.** It has carried `ToPython()` and `ToDocument()` since it was written, and
**nothing had ever called them**. The three constants - `DVInvalidIndex`, `DVDefaultColor`,
`DVZeroVector3D`, used by 142 members - were already saying what their Python and their document form
is, while the generators threw that away and reconstructed it from the C++ string. Half of what
looked like new work was a function call.

**The other half is real**, and the audit's number holds: 260 defaults are raw C++ source text with
no Python form anywhere, in 95 distinct spellings. They are now translated by named tables -
`emptyContainerValues` (17), `namedDefaultValues` (7), `innerDefaultValues` (1) - with one rule for
`Name({...})` and one for `Type::Value`. **An expression no rule covers raises
`UnknownDefaultValue`**, which stops the generator and names the table to extend. That is the whole
difference from the old converter: it guessed, and a guess that is wrong looks plausible.

**And the third finding: the corruption was published.** The audit said the `f`-removal was
"contained by accident" because its three victims are `CFNoInterface`. Writing the tables out showed
what the audit had not looked for - what the **document** converter does to the same values:

| in `docs/generated/` | was | is |
|---|---|---|
| 85 cells | `[ invalid [-1], invalid [-1] ]` | `[ invalid (-1), invalid (-1) ]` |
| 30 cells | `Matrix[]`, `PyMatrixContainer[]`, `MatrixI[]` | `[]` |
| 1 cell | `[Matrix3DF[3,3,1.,0.,0., 0.,1.,0., 0.,0.,1.]]` | `[[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]` |
| `itemInterface.py` | `coefficientsHull =  []` | `coefficientsHull = []` |

The nested brackets are the blind `"(" -> "["`; `Matrix[]` is the same replacement eating a paren in
a type name. The rotation matrix is the best of them: its own description says *"in python use e.g.:
initialModelRotation=[[1,0,0],[0,1,0],[0,0,1]]"* directly beside a default value that had been put
through two string converters.

**`grep` had found none of the string-valued ones, and the reason is in the generator**:

```python
if parameter['type'] != 'String' and parameter['type'] != 'FileName': #don't do this for file names, because 'f' is erased!
```

Somebody met the f-removal years ago and **worked around it by excluding two types**, in a comment
that says exactly what is wrong and was never acted on. `solverInformation.txt` and `images/frame`
are safe on the pages because of that line, not because the converter was right. The line is gone.

**Four functions and 201 lines went**: `DefaultValue2Python`, `Str2Latex`, `SplitString`,
`CutLinesFromString`, and the 14 `Str2Latex` calls that RG3.24.2 measured at 0 changes out of 3720
inputs. **The regeneration after the removal produced the same 47 files and the same 74 lines as
before it** - the measurements were right, and running the gate twice is what proves it rather than
says it.

**No regular expressions in a definition file.** The first version used `re`, and
`checkDefinitions` rejected it: it reads every string literal of `definitions/` and a backslash
followed by a letter is a LaTeX command to it, so `\\s` and `\\t` were four findings. That check is
right - a description is what those files are for - so the patterns became plain string logic, which
also made the f-suffix rule something a reader can see:

```python
if (character in 'fF' and index > 0 and characters[index - 1] in _numberEnd
        and not (following.isalnum() or following == '_')):
    continue                                     #a float suffix: it goes
```

**17 tests**, in `python/testing/test_defaultValueRenderings.py`, each named after the defect it
forbids - `testTheLetterFInsideANameSurvives`, `testAFileNameKeepsItsF`,
`testAnUnknownExpressionRaisesInsteadOfBeingGuessedAt` - and one that renders all 1376 defaults, so
a missing rule is a test failure and not a surprise during a release.

**RG3.24.2 got one verdict wrong and it is corrected in the plan**: `GetTypesStringLatex` does *not*
write a macro that its callers strip. It writes `\\texttt{...}` into text that goes through the
LaTeX-to-Markdown converter, which turns it into a backtick span - which is why none reaches a page.
The function is correct; only its name is wrong, and that belongs to RG3.24.

**What was deliberately not done.** The three constants' document wording disagreed with what is
published - `'invalid index'` against `invalid (-1)`, prose against `[-1.,-1.,-1.,-1.]` - and the
published wording won; the constants were corrected to it. This step removes corruption, it does not
re-word the manual. Whether a default column should read `exudyn.InvalidIndex()` rather than
`invalid (-1)` is now a one-line decision instead of an archaeology.

**Gates**: 11/11 checks, the wheel, the full suite, 464 pytest, the strict HTML build and the PDF.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.10 - the workflow of the override settings (2026-09-26, #2684, and RG12.5.4 with it)

RG12.9 put the values in `exudyn.special.overrideSettings`. This makes them arrive where a user meets
them, in the six steps the maintainer wrote out - which are now also what the documentation says,
because a workflow that is not written down is one nobody uses on purpose.

**A stored `visualizationSetting` reaches every structure that is created.** Only a
`SystemContainer` applied them, through a Python subclass; `exu.VisualizationSettings()` got nothing,
so a script that edited a structure before creating a container saw the defaults - the opposite of
what the file is for. There are now two subclasses, installed **only** when the file holds a
`visualizationSettings` section, so a user without a file gets the compiled classes untouched.

**The trap, which is the reason this step is worth reading.** A Python subclass changes what
`type(structure)` is, and `settingsUtilities.DefaultSettingsDictionary` is
`type(settingsStructure)().GetDictionaryWithTypeInfo()`. On an instance of the override-applying
subclass that constructs **the subclass**, applies the overrides to it and reports them as the
DEFAULTS. Measured on the first attempt: `openGL.multiSampling` came back with **4** as its own
default, where the default is 1. Everything that shows a difference runs through that function - the
dialog's *changed* marking, its *diff to default*, `ChangedSettings`, `Store(SC)` - so the whole
mechanism of "what did I change" would have quietly started comparing the overrides with themselves,
and `Store(SC)` would have stopped storing the settings a user had just set.

`CompiledSettingsClass(structure)` walks `type(structure).__mro__` to the first class the compiled
module defines, found by `exudyn._compiledModule.__name__` - **not** by
`type(exu.SystemContainer).__module__`, which is the metaclass and says `pybind11_builtins`; that was
the first attempt and the measurement caught it too. A structure that is not a subclass is its own
compiled class, which is the normal case and costs one loop.

**The results monitor has no file of its own.** The plan had a migration; the maintainer answered:
*"I just deleted the resultsMonitor.json file. It shall not be used any more. Everything inside the
new file. It also disappears from docs - it was just here for a few hours."* So `SettingsFileName` is
gone, `LoadSettings` and `SaveSettings` read and write the `resultsMonitor` section, and
`docs/manual/resultsMonitor.md` names `~/.exudyn/config.json`. **RG12.5.4 closes with it**, without
the migration it planned - the cheapest way to finish a step is for the thing it was careful about to
stop existing.

**One writer per section.** `overrideSettings.StoreSection(name, values)` merges one section into the
file and updates the store, and refuses a section that is not in `sectionNames`. `StoreDialogGeometry`
did it by hand and now goes through it, and so does the monitor: three writers became one.

**And one record per setting, not one per structure.** `_Record` appended on every application, and
the settings are now applied again for every structure that is created, so `Print()` would have
listed the same setting once per structure - which says how many structures exist, not what was
stored. It de-duplicates.

**Checked by hand, because the import-time path is over before a test runs**: a file holding
`outputPrecision`, two `visualizationSettings` and a monitor setting, with `EXUDYN_CONFIG_FILE` in
the scratchpad - `exu.config.outputPrecision` 9, `exu.VisualizationSettings()` 4 and 0.5,
`SC.visualizationSettings` 4, the default still 1, the monitor 3.0, `StoreSection` keeping the other
sections, and `Applied()` holding 3 records for 3 settings across two structures.

**One conversion detail for the next person writing into `pb.AddDocu`**: `\\ben ... \\item ... \\een`
reaches the page **as itself**. `AddDocu` goes through `autoGenerateHelper.LatexText2Markdown`, not
through `latexToMarkdown.ConvertText` which the item and structure descriptions use, and that one
does not know the list macros. Plain Markdown survives it - six `<li>` in the built HTML - and it is
what the rest of `pybindModule.py` writes.

**Gates**: 11/11 checks, the wheel, the full suite, 470 pytest, the strict HTML build and the PDF.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.11 - storing the override settings (2026-09-26, #2685)

Three defects, and all three were the same one: nothing knew the defaults.

**`exudyn.config` had none, and they cannot be constructed.** Every settings structure finds its
defaults by `type(structure)()`; `ExudynConfig` is a **facade over global variables**, so a second one
reports the **current** values - measured while planning: after `exu.config.outputPrecision = 12` a
freshly constructed `Config` says 12. That is why `Store(config=...)` guessed: *"every value that is
not `''`, `0` or `False`"*, which stored `printToConsole` from every run that never touched it and
`outputPrecision` because 6 is not 0.

The maintainer chose the snapshot: `EPyUtils::ConfigDefaults()` is filled in
`Init_Pybind_manual_classes` from `pyConfig`'s own getters, **after** the class is registered - the
first attempt put it before, where `py::cast(&config)` has nothing to cast to - and before any user
code, any override setting or any environment variable can change one. **What Exudyn starts with is
the default**, taken while it still is: nothing is declared twice and nothing can drift out of step.
`GetDictionary()`, `SetDictionary(d)` and `GetDefaults()` are the interface the maintainer asked for,
and the ten settings are listed **once**, in `configSettings`, with the three that only report
(`printToFile`, `printFileName`, `printToFileAppend`) marked, so a new setting of `exudyn.config`
reaches the dictionary, the defaults and everything comparing against them by being added in one
place. A test requires the dictionary to hold exactly what the Python interface holds.

`Main/Config.h` stays free of pybind11, as `Main/Experimental.h` did in RG12.9: eight translation
units include it, two of them in `Linalg` and `Utilities`. The functions are declared in
`PybindUtilities.h`, which already has pybind, with a forward declaration of the class.

**The store button.** It writes the `visualizationSettings` that differ from the defaults and the
dialog's own size and position, and **nothing else**. One click would otherwise reach the home
directory, so it opens the window `ShowCodeLines` already builds - with the exact lines - and a
**store / cancel** pair; `ShowCodeLines` gained one optional argument for that and every other caller
is unchanged. It stores the geometry whether or not `storeDialogPositions` is on: that flag decides
whether a dialog remembers itself when it closes, and this is a user asking. The geometry parser moved
out of `StoreWindowGeometry` into `StoreGeometryString(geometry, name)`, so there is one parser and
the button does not need a recorded dictionary.

**And "diff to default" stays a difference to the default**, as decided: a stored setting **is** a
difference and is listed as one, and the settings the file already covers are named again under
`#the following are already stored in ...`. Comparing against default-plus-override would hide
exactly the settings that file is about. The grouping is `GUI.SplitStoredFromChanged`, a module-level
function rather than four lines inside the handler, because a handler that opens a window cannot be
tested and this can: two tests, one of them asserting that nothing is dropped and exactly one comment
is added.

**One bug of my own, found by reading what RG12.10 had just changed.** The first version asked
`isinstance(self.settingsStructure, exudyn.VisualizationSettings)` to decide whether the override
section applies. Since RG12.10 that name is the override-applying **subclass** whenever a settings
file exists, and `SC.visualizationSettings` - an instance of the compiled base - is **not** an
instance of it, so the dialog a user is most likely to have open would have grouped nothing. It asks
`CompiledSettingsClass(...)` instead, which is the helper RG12.10 added for the same reason. A
mechanism that changes what a class is has to be remembered by everything that asks what a class is.

**What was not verified by a test, and is stated rather than implied**: the button was never clicked.
Its two halves are tested separately - `SplitStoredFromChanged` and `StoreGeometryString` with a
temporary settings file - and the writing goes through `StoreSection`, which has its own tests, but
no test opens the dialog and presses it. It wants one click from the maintainer.

**Gates**: 11/11 checks - `checkAll` asked for `__all__` in definition order and rewrote it itself -
the wheel, the full suite, 481 pytest and the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.13 - a stored dialog geometry is used (2026-09-26, #2686)

The maintainer tested 1.12.95: *"it saves the settings and the positions. However, when loading and
opening the vis settings, it reports an error. When I remove the second 'visualizationSettings', it
loads, but position is not restored."* Two symptoms, and only one of them had a cause that could be
found.

**The position: my own defect from RG12.11, one commit old.** `RestoreWindowGeometry` began with

```python
if not StoreDialogPositions():
    tkWindow.geometry(str(width) + 'x' + str(height))
    return
```

so the geometry the **store button** writes is never read - and the button exists precisely so that a
user does not have to switch `dialogs.storeDialogPositions` on. I built the button and left its reader
gated by the flag it was designed to avoid. Measured with the maintainer's file: the window is asked
for `900x700` while `1122x1751+7+14` is stored; after the change, for `1122x1751+7+14`. The same probe
found it and confirmed the fix, which is the only reason the fix is believable.

**The flag now decides what its name says**: whether a dialog stores *itself* when it closes.
`RememberWindowGeometry` still asks it, which is where it belongs; `RestoreWindowGeometry` only reads
what is there, whether it is there because the flag was on, because the button wrote it, or because a
script did.

**And a second, wider half, found while checking the first.** `StoreDialogPositions()` asked
`GetRendererSystemContainer()`, which is **None whenever no container is attached to a running
renderer**. So for `python -m exudyn dialogs` - which has no container at all, as its own docstring
said - and for any script that opens a dialog before `renderer.Start()`, the flag was False however
the user had set it, and such a dialog could **never** store itself. It now asks the structure being
edited first, which is the one the user is looking at.

**A stored size is cut down to the current screen.** This came out of the measurement rather than the
report: the maintainer's stored height is **1751** pixels, and on a 1234-pixel screen the dialog's
bottom - the button row, with *close* in it - is below the edge. The rule for the position exists for
exactly this ("a dialog that cannot be closed is a stuck session", RG6.2.11) and the size had no rule
at all. It has one now, `dialogScreenMargin` = 40 pixels on each side, and on the maintainer's own
screen, where 1751 fits, nothing changes.

**What could not be reproduced, and is not pretended otherwise.** The *error* was chased with their
exact file: through the import, through `Print()`, through `DialogGeometry` for three spellings of the
name, and through the same settings dialog built in a withdrawn window as `test_guiValues.py` builds
it - 470 settings, no exception, every reader returning what it should. They have since deleted the
file, *"as it might have been in an invalid state"*, so there is nothing left to chase. **RG12.13.1**
stays open so that a recurrence is recognised instead of investigated from the beginning.

**Two of the maintainer's other questions were answered by measuring rather than by planning**, and
both made a step smaller:

- *"For RG12.14 / Spyder: it seems that StoreSection already stores the values not only in the file,
  but also in the local settings. This would already be exactly what we need, right?"* - Yes, and
  more: because the subclasses of RG12.10 close over **that dictionary object**, a structure created
  after a store gets the new value in the same session. `general.circleTiling` stored as 7 reads back
  as 7 from a new `exu.VisualizationSettings()` and a new `SystemContainer`, with no restart. What
  looked like "it does not store" was this step's defect.
- *"For RG12.15: this could also be done by directly writing into exudyn.special.overrideSettings,
  right?"* - Yes, and it does **not** touch the file, so it places a dialog for one run; a reader with
  a rule refuses a bad value (a size of `['wide', 'high']` comes back as `None`), which is one
  reader's rule and not a promise, which is why it is the footnote and the functions are the
  documented way.

**Tests**: five, and they measure what the function **asks the window manager for**, because a
withdrawn window reports `1x1+0+0` whatever it was given. They use `TkRootOrSkip` rather than the tree
fixture, so they run under xdist as well.

**Gates**: 11/11 checks, the wheel, the full suite, 486 pytest, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.17 - a settings file killed the V key, and RG12.13.2 - a version in the file (2026-09-26, #2691, #2690)

The maintainer tested 1.12.96 properly: deleted the file, ran a model, placed the dialog, stored,
reopened it in the same session - *"so this seems to work already"* - then restarted the session and
pressed V. Nothing opened, and the console said *"problems with the SystemContainer, probably not
attached to the renderer yet"*.

**The cause, reproduced in twenty lines.** `GetRendererSystemContainer()` does

```python
isinstance(guiSC, exudyn.SystemContainer)
```

on what the C++ side stores - and the C++ side stores a **pointer**:
`exudynModule.attr("sys")["currentRendererSystemContainer"] = this`, which pybind casts to an object
of the **compiled** class. RG12.10 had installed a Python **subclass** under the name
`exudyn.SystemContainer`, and a compiled-class object is not an instance of it. So the probe returned
None and every dialog that needs the container - the settings dialog on V, the right-mouse dialog -
did nothing. Only with a settings file, which is why it worked before the maintainer stored anything.

**This was the third thing that subclass broke in two days**, and that is the finding, not the fix:

| when | what it broke |
|---|---|
| RG12.10 (#2684) | `DefaultSettingsDictionary` constructed the subclass and reported the overrides as the defaults |
| RG12.11 (#2685) | an `isinstance` of mine stopped recognising `SC.visualizationSettings` in the dialog |
| here (#2691) | `GetRendererSystemContainer()`, in code nobody had touched since 2023 |

The first two I found myself, and each time I fixed the *caller*. The third one a user met. **A
mechanism that changes what a class is cannot be made safe by fixing the places that ask what a class
is**, because that set is unbounded - it includes every line of Exudyn, every example and every user
script.

**So the class is no longer changed.** `pybind11` heap types allow their `__init__` to be wrapped in
place - measured before relying on it - so the constructor of the **compiled** class applies the
stored settings and `exudyn.SystemContainer` stays what it always was. No subclass, no name moved, no
`isinstance` to fix. If a build ever refuses the patch, it says so and applies nothing, rather than
falling back to the subclass that broke the dialogs.

**And that cost one thing, which the same session had already solved elsewhere**: with the compiled
constructor applying the overrides, the defaults cannot be **constructed** any more - `CompiledSettingsClass`
was no help, because the compiled class is the one applying them. They are **snapshotted at import**,
before the wrapper is installed, and `DefaultSettingsDictionary` hands out a copy. That is the same
decision as for the defaults of `exudyn.config` in RG12.11, for the same reason: what Exudyn starts
with is the default, taken while it still is.

**Two things the maintainer asked for while reading the fix, and both were right:**

- *"I very much believe that GetRendererSystemContainer() dies without a message - pass could instead
  write at least a message to know that it happened and where."* It now says, **once per process**,
  what went wrong: a wrong class, a container whose C++ object is gone, an exception. The normal
  answer - no renderer - stays quiet, because every dialog asks. Had that message existed, this bug
  would have been a line of output instead of a reproduction.
- `ShowVisualizationSettingsDialog` printed **"ERROR: ShowRightMouseSelectionDialog: ..."**, a
  copy-paste error naming the wrong function in the one message a user sees - which is also why the
  report named a dialog nobody had pressed.

**What was NOT wrong, and is worth writing down because it looked wrong**: the *"strange position
coordinates"*, `[-10, 0]` for a window at the left edge. That is Windows reporting the invisible
resize border, and `PositionIsReachable` already allows `screenX - 16` for exactly that - the rule's
own comment says *"a maximised window sits at -8"*. The position is restored.

**RG12.13.2, the version, in the same commit and to the maintainer's specification**: *"just add a
version number 1 for now ... Very simple, no deep tech; similar as in FEM."*
`overrideSettings.fileFormatVersion = 1` is written by `Save`, always the current one whatever the
file said; `Load` **ignores** a file whose version does not match - including one with no version,
which is every file written before today - with one note naming both and saying to store the settings
again. It is never a section and never reaches the store. The test fixture writes it too, which is how
the three tests that broke on it were the right kind of failure.

**Gates**: 11/11 checks, the wheel, the full suite, 494 pytest, the strict HTML build. The V key
itself was not pressed by me - it needs a render window - so the reproduction is the probe the C++
side feeds, and it fails before the fix and passes after it.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.18 - the renderer link is a member on the C++ side (2026-09-26, #2692)

The maintainer, having read the fix of RG12.17: *"exudyn.sys['currentRendererSystemContainer'] stores
the currently active SystemContainer (as an old, dirty hack), as glfw can only hold one at a time. =>
however, we can just store it on the C++ side of the code - module-wide, like in
special.currentRendererSystemContainer. => this would immediately return the correct link."* And:
*"file this as next step or do it immediately, before doing strange workarounds."*

**The strange workaround was mine**, one commit earlier: `isinstance(guiSC, <the compiled class>)`
existed only because a dictionary entry can hold anything. A member cannot be wrong about its own
type, so the check is gone rather than made cleverer.

**A raw pointer, not a Python object.** `MainSystemContainer::currentRendererContainer` is a
`MainSystemContainer*`, set when a container attaches and cleared when it detaches, and
`exu.special.currentRendererSystemContainer` casts it on access - which returns the Python object that
already wraps that pointer, so a script gets the very container it created. Nothing holds a Python
reference, so the lifetime rule of RG12.9 - a global must not release a Python object after the
interpreter has finalized - does not even come up.

**It fixes something the entry never did, and that is the part worth reading.** The destructor calls
`Reset()`, and `Reset()` called `visualizationSystems.DetachFromRenderEngine(...)` **directly** - not
`DetachFromRenderEngineInternal`, which was the only place that cleared the entry. So a destroyed
container left `exu.sys` naming an object whose C++ side was gone, and it was the first **use** of it
that raised, in whichever caller happened to be next. That is #2623 and #2676: two issues spent
guarding the *reader* against a link that the *writer* should never have left behind. The pointer is
cleared in `Reset()`, so the reader gets `None`, and the probe that remains is a cheap second belt
rather than the only one.

**Three readers became one.** `GetExudynDisplayScaling` and the dialog's redraw read the link
themselves, so the dangling guard applied to one of the three. Both call
`GetRendererSystemContainer()` now. A test can also replace that one function, which it has to: the
link belongs to the C++ side and Python cannot take it away - one test used to `pop` the dictionary
entry to reach the tkinter branch, and it now patches the reader.

**What used it, which the maintainer asked to check**: `exudyn.misc.GUI` (three readers), the
`basicUtilities` workspace clear, two test files, and the C++. **No example, no test model, no
documentation page.** The workspace clear is the one behaviour change worth naming: it emptied
`exudyn.sys` and so took the renderer's link away as a side effect, and it does not any more, which is
what a reader of that function would expect.

**And one thing was reverted on the maintainer's word**, from RG12.13.2 in the previous commit: the
code no longer says anything about settings files written before the version existed. *"the config
file was just alive a few hours, we don't track something like that in the memory of the code."* The
rule stands - a file carries `version` 1 or it is not read - and the code says the rule and nothing
about this morning.

**Gates**: 11/11 checks, the wheel, the full suite, 495 pytest, the strict HTML build. The V key needs
a render window, so what is tested is the link: `None` without a container, the container itself after
one is created, absent from `exudyn.sys`, and not settable from Python.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.14 - the override settings can be read again, and RG12.16.1 - the render window can be placed (2026-09-26, #2687, #2689)

**RG12.14 was two thirds finished before it was built, and not by me.** The maintainer, reading the
step: *"For RG12.14 / Spyder: it seems that StoreSection already stores the values not only in the
file, but also in the local settings. This would already be exactly what we need, right?"* Yes - and
measured, one better: because the wrapped constructors of RG12.17 close over **that dictionary
object**, a structure created after a store gets the new value in the same session. So what looked
like *"it does not store"* was RG12.13's defect, and this step is only what is genuinely left: a file
edited **outside** the session.

`overrideSettings.Reload()` empties the store, fills it from the file, applies `config`, and - the
case that needed thought - **installs the wrapped constructors if the file had no
`visualizationSettings` at import**. That was the fork in the step: install them always (a cost for
everybody), install them on a reload, or document that this one case needs a restart. The reload
installs them, which costs nothing in a session that never reloads and makes the reload complete.

**A reload does not undo, and that is written where a user reads it.** A setting that already reached
`exudyn.config` stays; a structure that exists keeps what it was given. What a reload *does* do, which
I expected not to, is that a section **removed** from the file stops reaching new structures - the
store is emptied and the wrapper then applies nothing. Measured: `general.circleTiling` 4, remove the
section, reload, a new structure reads 16 again. The probe I wrote said "(4 expected)" and the code was
right.

**And the snapshot survived the same test**: the defaults still read 16 after the reload installed the
wrappers, because the snapshot is taken before they are installed. I had annotated that probe line
wrongly too.

---

**RG12.16.1: the render window can be placed**, and the maintainer's design made it small - an
ordinary setting, so `~/.exudyn/config.json`, `Store(SC)` and the store button carry it with nothing
added, one entry per view, and a script can set it. `GlfwClient.cpp` calls `glfwSetWindowPos` when it
is asked to; it never called it at all before.

**The `(-1,-1)` sentinel could not be used, and the reason is the interesting part.** The maintainer
proposed it with a question mark - *"default needs to be something illegal (-1/-1)?"* - and the
question has an answer: the settings dialog **refuses a negative value of an `IndexArray`**, so the
value could be stored in a file and set from a script but never typed in the dialog. It is the same
C++ type as `renderWindowSize` beside it, so no mapping can tell the two apart.

I tried the obvious way out - let a **fixed-size** `IndexArray` be signed while a variable-length one
stays non-negative, on the grounds that a list of indices is item numbers and a pair is geometry - and
an existing test said no: `('[-1, 2, 3]', 'IndexArray', [3], 'positive integer')` is a case somebody
wrote down on purpose. Relaxing the rule for all of them was the other way out, and measuring killed
it: `v.sensors.traces.listOfPositionSensors = [-1]` and `renderWindowSize = [-5,-5]` are **both
accepted by the C++**, so the dialog's rule is the only one there is.

So "unset" is a **flag**, `useRenderWindowPosition`, default False. Two settings instead of one, which
is more than was asked for, and the only option that plays by the rules that exist rather than by one
I invent. If the sentinel is preferred, it needs a distinct type for a signed pair - a bigger change
than the flag, and RG12.16.2 is where it would go.

**The reference of `parameterConversionTest` was rewritten**, as its own header says to do after an
intended change: 16 rows, all of them the two new settings appearing in the four views' lists, and
nothing else moved.

**Gates**: 11/11 checks, the wheel, the full suite, 498 pytest, the strict HTML build. The window is
not placed by any test - that needs a render window - so what is pinned is the contract: the flag is
off on all four views, the position defaults to (0,0), and both are ordinary settings that
`ChangedSettings` reports.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.16.2 and RG12.16.3 - the render window and the SolutionViewer remember themselves (2026-09-26, #2689)

**RG12.16.2**: `view*.window.storeRenderWindowGeometry`, off by default, writes the size, the position
and `useRenderWindowPosition` back into the settings when the window closes - so *store settings* in
the visualization settings dialog keeps a render window where it was left, instead of the user reading
coordinates off the screen and typing them in.

**Three things about it were decided rather than assumed:**

- **Off by default**, for the reason `dialogs.storeDialogPositions` exists: a settings structure that
  changed by itself would make *diff to default* report a window size and position after **every**
  run, and a user storing settings for an unrelated reason would pin their window without meaning to.
- **It sets `useRenderWindowPosition` too.** Writing a position that nothing then uses would be a
  setting that looks stored and does nothing - the mistake of RG12.11, where the store button wrote a
  geometry the reader refused to read.
- **`glfwGetWindowPos` and `glfwSetWindowPos` are both about the content area**, so what is written
  back is exactly what `CreateViewWindow` reads - unlike a tkinter geometry string, which carries the
  frame and is why the maintainer's stored dialog position reads `-10` on Windows.

The write goes through `GetSettingsViewWritable(...)`, a **deliberately differently named** accessor
rather than a `const` overload of `GetSettingsView`: the renderer writing *into* the settings happens
once, when a window closes and only because the user asked, and a call that does that should not look
like the ordinary read. An overload would also have quietly changed which function forty existing call
sites resolve to.

**RG12.16.3 was the cheap half and got cheaper.** The SolutionViewer's window is an
`InteractiveDialog` - and so are `AnimateModes` and an interactive simulation - so putting
`RestoreWindowGeometry` at the end of its constructor and `StoreWindowGeometry` in `OnQuit` makes
**all three** remember themselves, each under its own title, through the same `dialogs` section of
`~/.exudyn/config.json` as the settings dialogs, with the same reachability rule and the same flag
deciding whether a dialog stores itself on closing.

One thing had to give: `RestoreWindowGeometry(window, name, width, height)` always imposed a size, and
an `InteractiveDialog` computes its size from its widgets. The width and the height are optional now,
and with nothing stored and none given the function **returns without touching the window** - the
layout decides, which is what it did before. A test pins that, because "the dialog is suddenly 900x700"
would be a regression nobody would attribute to this step.

**And a reminder that the tests read the INSTALLED package**: the new test failed with
`bad geometry specifier "NonexNone"` until the wheel was rebuilt - the source had the guard, the
installed copy did not. The gate order in the workflow says to build first, and it says it for this
reason.

**The reference of `parameterConversionTest` was rewritten twice** in this pair of steps, once per new
setting, exactly as its header prescribes: 16 rows then 8, all of them new settings appearing in the
four views' lists, nothing else.

**Gates**: 11/11 checks, the wheel, the full suite, 499 pytest, the strict HTML build. Neither window
is opened by a test - both need a real window manager - so what is pinned is the contract around them:
the flags are off, the position defaults to (0,0), a stored geometry is used, and a dialog with nothing
stored is left alone.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.12, RG12.15, and the flag the maintainer measured away (2026-09-27, #2688)

**First a correction to yesterday's step.** RG12.16.1 shipped with a `useRenderWindowPosition` flag
because the settings dialog refuses a negative `IndexArray` and I could not tell the position from the
size. The maintainer went and **measured the window itself**: a GLFW window position is always
positive, *"so this means that we CAN take the negative values (any of both)"*. The flag is gone and
`(-1,-1)` is the sentinel again, which is what they proposed in the first place.

What remains of my objection is one line in `knownRoundTripGaps` - until now an empty list - naming the
four paths the dialog cannot type a negative value into, with the reason: a user **sets** a position
there, which is positive and passes, and unsets it in the file or from a script. A gap that is named is
not the same thing as a gap that is hidden, and the test would report it if it ever closed.

They also measured what the number means, which is worth more than the flag was: it is the position of
the **OpenGL area**, not of the title bar, so a value below about 50 hides part of the title bar and 0
hides it completely - *"this works, as there is still the escape button"* - and that can be wanted. The
description says so now.

---

**RG12.12, the defaults**: eleven arguments of `PlotSensor` defaulted to the same literal as
`PlotSensorDefaults()`, and the code decided "the user did not pass this" by comparing against that
literal - `if fontSize == 16`. So passing 16 on purpose was indistinguishable from not passing it, and
the function's own documentation said so out loud:

```
#==>BUT PlotSensor(..., fontSize=16) will use fontSize=12, BECAUSE 16 is the original default value!!!
```

A docstring that explains a defect is a defect with a witness. The arguments default to `None` now,
`None` is what asks for the default, that sentence is gone because the behaviour is gone, and the
mutable default arguments (`colors=[]`, `sizeInches=[6.4,4.8]`) went with it - a Python wart that was
only invisible because nothing mutated them.

**The file**: a `plotSensor` section sets any of those defaults for every run, read when
`exudyn.plot` is imported, with the same discipline as the rest of the file - a name that is not a
default is reported, not invented.

**The window positions**, by the decision in the plan: **by their sequence**, the counter reset by
`closeAll=True`, because plot windows have no unique title. Two things fell out of writing it that the
plan had not said:

- **only the position.** A plot's size is `sizeInches`, which is already a default this file can set -
  storing a *pixel* size beside it would be two sources for one thing, and the pixel one would win by
  accident.
- **it is all borrowed.** The `dialogs` section, `DialogGeometry`, the reachability rule and
  `StoreGeometryString` all work unchanged for a matplotlib window, because TkAgg's `window.geometry()`
  returns the same `'WIDTHxHEIGHT+X+Y'` a dialog gives. Qt gets its own two lines; any other backend is
  left alone.

**No test opens a plot window**, so that half is a contract and not a measurement, and it is off by
default - `PlotSensorDefaults().storeWindowPositions` - so nothing changes for anyone who does not ask.

---

**RG12.15** is the documentation the maintainer asked for: placing a dialog from a script with
`StoreDialogGeometry(name, size, position)`, placing the render window with its own two settings, and
the footnote about writing into `exudyn.special.overrideSettings` directly - which works, holds for one
run, does not touch the file, and is **not** recommended, because nothing checks what is put there. It
waited for RG12.13, because an example of placing a window that nothing reads back would have been
worse than no example.

**Filed rather than built**, both from the maintainer while this was being written: **RG12.19** (#2693)
two buttons, one for the settings and one for the positions - *"they are two decisions"* - and
**RG12.20** (#2694), the one real hole this family has left: the render window's geometry lives in the
view settings *and* in the file, they are applied at different moments, a script already wins by
ordering, and nobody is told when the file said something else. That wants a step of its own, and it
ends in the documentation.

**Gates**: 11/11 checks, the wheel, the full suite, 499 pytest, the strict HTML build. The reference of
`parameterConversionTest` was rewritten once more, for the setting that went and the one that came.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.19 and RG12.20 - two buttons, and where the render window is (2026-09-27, #2693, #2694)

**RG12.19** splits the store button in two, because they are two decisions - *I like this look* and *I
like this window here*. The question the step had left open was which button takes the **render
window** geometry, and the maintainer answered it in a way that needed no code: *"I opt to store it in
the config file in the visualizationSettings, because it is the straightforward way and becomes now
natural, because it is only stored if it differs from default."* It is a `visualizationSetting`, so it
rides along in the settings button and nothing special had to be written for it.

---

**RG12.20 was easier than the step I had filed, because the maintainer had read the code first**:
*"there is already SetRenderStateScreenSize in GlfwClient.cpp and it only needs to be copied or
extended to size AND position ... follow the trace of the state->currentWindowSize, to add a
currentWindowPosition to the RenderState, also making it read/write in the MainRenderer::Get/SetState."*
The trace was exactly that, in five places: the field, its initialisation from the setting, the refresh
from GLFW, the dict, and the dict's way back.

**No window-move callback was needed**, which the maintainer had flagged as a maybe. `Render` already
refreshes `currentWindowSize` on every frame, so the position is asked for in the same place: one
`glfwGetWindowPos` beside a redraw, and one place where the state learns about the window instead of
two that can disagree.

**`SetState` writes the setting as well as the state** - the maintainer's *"otherwise a re-open would
not have the just stored positions"* - and that was not new so much as **symmetric**: the size has
written `window.renderWindowSize` that way for years, and the position simply did not exist. The
write-back of RG12.16.2 now reads the state instead of asking GLFW itself, so the recorded geometry and
the state cannot differ.

**And the conflict is said out loud.** The render window is the only window whose geometry lives in two
places - the view settings, and the `visualizationSettings` section of the file, because those are
ordinary settings - and they are applied at different moments: the file when the structure is
constructed, a script afterwards. So the script already won; what was missing is that nobody was told.
`SC.renderer.Start()` now says it **once** per process, naming both values, and only when they really
differ:

```
Python WARNING: the render window geometry stored in the settings file differs from what this session
set, and what the session set is used:
  view0.window.renderWindowPosition: the file says [100,80] and this session uses [500,400]
store the settings again to change the file, or remove them from it
```

**One placement decision, and it was made by a failed test.** The check first sat *after* the
`suppressRenderer` guard, which is where a reader would put it - no window, no warning about a window.
Then the probe that was meant to prove it printed nothing, because the guard returns first. It is
before the guard now, and the reasoning changed with it: the disagreement is between the **file** and
the **session**, which is true whether or not a window opens, and `Start()` is simply the moment both
are known. With no settings file the function returns at once, which is every test run - so the noise
where it would matter is zero.

**What is tested and what is not**, said plainly: the render state carrying the position, and `SetState`
writing the setting, are two tests. The warning was verified **by hand**, with the output above: equal
values silent, differing values one warning, a second `Start()` quiet. No test opens a render window,
and none opens the dialog whose buttons these are.

**Gates**: 11/11 checks, the wheel, the full suite, 501 pytest, the strict HTML build.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG12.21 and RG12.22 - the info command and the monitor's focus (2026-09-27, #2695, #2696)

Two from the maintainer's queue that were decided in the asking, and one cause worth writing down.

**RG12.21**: *"python -m exudyn info: you mention that this shall be used when submitting issues. But
I believe that it should either not show the user path."* It printed the package directory, the Python
executable and the output directory, and on a normal installation all three are under the home
directory - so the one command whose whole purpose is to be pasted into a public issue carried an
account name. They are shown as `%USERPROFILE%` or `~` now, which is also what a reader would type
themselves, and `--showPaths` gives the real ones for a problem that is about a path.

**RG12.22 had a cause that explains both halves of the complaint at once**: *"always takes focus (so
one cannot use the control panel); always in front of terminal"*. The update loop called
`plt.pause(period)`, and `plt.pause` is:

```python
show(block=False)                 #<-- for TkAgg: deiconify() and lift()
canvas.start_event_loop(interval)
```

So the window was raised **and** focused once per update period - one to twenty times a second - which
is why the control panel beside the plot could not be typed into: the focus was taken back before a
keystroke arrived. The loop calls `start_event_loop` directly now, which is the half of `pause` that
does the waiting, and `plt.pause` remains as the fallback for a backend without an event loop.

*"except optionally alwaysOnTop"* is a monitor setting of that name, default False, and - being a
monitor setting - it is stored in the `resultsMonitor` section of `~/.exudyn/config.json` like the
rest, with `--always-on-top` on the command line. It puts the window above the others and still does
not take the keyboard.

**One thing had to be given up, and the gate caught it**: the Qt branch of `alwaysOnTop` needed
`QtCore.Qt.WindowStaysOnTopHint`, so it imported PyQt5 - and `checkExtras` refused the commit, because
that is a new dependency for one line and CLAUDE.md rule 6 says no. It is tkinter only and says so.
The plot-window placement of RG12.12 keeps its Qt branch, because `window.move(x, y)` needs no import.

**Also from the queue, without code:**

- **#2673 was clarified on request** - *"I don't understand any more what was the reason, and what is
  the content"* - and RG3.21 now says the reason, the scope (`docs/manual/` only, with the eight
  subjects named), the method (compare `revisions.md` and the release notes against the pages, rather
  than re-read the manual) and that the first deliverable is the **list**.
- **#2672 was remarked**: the maintainer retried the asynchronous monitor and it still fails, which is
  exactly what that issue is - the existence test stands before the waiting loop. RG11.3.1 is the step.
- **RG3.26** (#2697), **RG12.23** (#2698): filed with what was measured, see the plan.

**Gates**: 11/11 checks, the wheel, the full suite, 505 pytest, the strict HTML build. Neither the
monitor window nor the info command is opened by a human in the suite: the focus is tested by reading
the loop's own source for `plt.pause`, which is the thing that must not be there.

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
### RG11.3.1 - the monitor waits for the file (2026-09-27, #2672)

Reported twice, three days apart: *"ResultsMonitor: unfortunately does not work as you promised"*
(2026-09-26) and *"I am not sure, if MonitorResults should work async already in the files - I tried,
but it does not work"* (2026-09-27). It should, and the second report is why it was done tonight.

**The whole defect was four lines in the wrong place**:

```python
while fileName != '':
    if not os.path.exists(fileName):
        print('ERROR: file not found: ' + fileName)
        return None
    monitor = ResultsMonitor(fileName, settings)
    if not monitor.WaitForData(...):
```

`WaitForData` waits patiently for a header and a row - and was never reached, because the line above
it gave up first. `StartResultsMonitor` exists to start a monitor **before** the solver, both examples
do exactly that, and both printed "file not found" and exited.

**The waiting now covers the file itself**, and the caller's test survives only for `--once`, which
plots what exists and returns - waiting there would be waiting for nothing.

**The open question of the step - what `--wait N` means for a file that never appears - answered
itself once the waiting was in one place**: `waitTimeout` already meant "seconds to wait for the first
data row, 0 without limit", so it means the same for the file. 0 is what a monitor started before the
solver needs, and N gives up with a message naming the file.

**And one thing was added that the step had not asked for**, because waiting without limit has a
failure mode of its own: what is being waited for is **announced, with the file name**. Waiting
forever for a file that will never appear is exactly what a typo in the name looks like, and a line
saying which file it is waiting for is the difference between a hang and a hint.

**Four tests**, and the first one is the promise itself: another thread writes the file after 0.4
seconds - the solver's part - and the monitor has to pick it up. It does, 0.16 s later, which is the
polling interval. The others: a file that never appears is an error after the timeout and not before
it, a file that is already there is not waited for at all, and a file whose first line is complete and
is not an Exudyn header is refused rather than waited for - that last one was already true and is now
guarded.

**Gates**: 11/11 checks, the wheel, the full suite, 509 pytest, the strict HTML build.

<a id="rg3-21"></a>
### RG3.21 — the pages that still described August (2026-09-27, #2673)

**The list was made by measurement, not by reading**, which is what made the step cheap. Two scripts:
one collects every `exudyn.x`, `exu.x`, `mbs.x` and `SC.x` identifier out of the fenced code and the
backtick spans of `docs/manual/`, imports the package and asks whether the name exists; the other
collects every settings path and assigns it under `warnings.catch_warnings(record=True)`, so a
deprecated one says so itself instead of being compared against a list somebody has to maintain.

**What the measurement said:**

| what was checked | how many | wrong |
|---|---|---|
| identifiers named in the manual | 496 | **3** |
| settings paths assigned under captured warnings | 129 | **0** |
| deprecated module-level call forms | 11 | **11** |

**The three wrong names**: `raytracer.imageSizeFactor` (the setting is under `openGL.raytracer`),
and two that were spelled as the module attribute of a `SystemContainer` method. **Zero deprecated
settings paths** is the interesting number: the manual was suspected of teaching settings that have
moved, and it teaches none - the measurement replaced a suspicion with a fact.

**The eleven were all the same thing**: `exu.SolveDynamic(mbs, ...)`, `exu.SolveStatic`,
`exu.ComputeLinearizedSystem`, `exu.ComputeODE2Eigenvalues` - the module-level forms that warn since
revision2026, written in `solver.md` as *the* list of the solvers, in a FAQ snippet **and in the
traceback printed under it**, and in the tutorial. A reader who copies the tutorial gets a warning
for something the tutorial told them to write. They are `mbs.` methods now, everywhere in the manual.

**Two hits are left in the list on purpose**: `exudyn.artificialIntelligence`, whose module exists but
does not import without the optional Gym and stable-baselines, and `mbs.Create...(...)`, which is a
family and not a name. A measurement that reports two known false positives is more useful than one
that hides them, so the two are recorded here rather than filtered.

**The prose the scripts cannot see** was read for the subjects the step lists, and one page was
**made stale by this week's own work**: `GUI.md` described *"the two buttons beside the copy button"*
and the dialog has four since RG12.19 and RG12.20. It now says what all four do, and what the
storing rules are - including that a stored geometry is used whenever a dialog opens, whatever
`storeDialogPositions` says, which is the distinction RG12.13 had to fix in the code.

Also corrected while walking the same pages: `resultsMonitor.md` (the suppress flag's name, the focus
paragraph of RG12.22, the waiting of RG11.3.1), `performanceErrors.md`
(`solutionSettings.writeSolutionToFile`, and a heading that promised behaviour rather than causes),
`introductionAdvanced.md`, `introductionBasics.md`, `tutorial.md` and `gettingStartedFAQ.md`.

**The step asked for the list first and the list decided the rest**: one commit, because what it found
was small. The two scripts are the thing to keep - they are the cheap way to ask the same question
again after the next group of steps, and the answer is a number.

<a id="rg12-23"></a>
### RG12.23 — the plot windows are stored while they are open (2026-09-27, #2698)

**What was missing was a moment, not a mechanism.** RG12.12 already placed a plot window where the one
of its sequence number was left and stored it when it closed. The maintainer pointed at when that
happens: *"the sensor windows are opened at a point when the renderer is usually already stopped"*, so
the settings dialog and its **store positions** button are gone, and a user who has just arranged four
plots has nothing to press. Storing on close also stores whatever a window happened to be at the
moment it was closed, one window at a time.

**`exudyn.plot.StorePlotWindowGeometry()`** is the answer, and it returns how many windows it stored.
It needs the list the maintainer described: `__plotWindowFigures`, `[(sequenceNumber, figure)]`,
appended to when `PlotSensor` places a window and cleared by `PlotSensor(..., closeAll=True)` - the
same moment the sequence counter starts over, because the two have to agree or a window is placed under
another window's name.

**A closed figure is recognised by its window being gone**, not by a flag: `figure.canvas.manager` has
no `window` attribute once the backend window is destroyed. So the list prunes itself every time it is
used - the store call skips such a figure and drops it - and nothing has to be unregistered.

**The size is now restored as well**, which the close handler alone could not make useful: a stored
geometry gives `WIDTHxHEIGHT+X+Y` to tkinter, or `resize`/`move` to Qt, and the position still only
when it would be reachable on the screen you have now. `sizeInches` stays what a figure gets when
nothing is stored.

**Tested without opening a window**, which is the only way a test of this can run in the suite: the
suite runs matplotlib on `Agg`, where a figure has **no window at all** and the whole mechanism is
correctly inert. So the store path is exercised with a stub figure whose window answers `geometry()`,
exactly as tkinter's does - five tests: nothing to store is not an error, two windows are stored under
their sequence names, a closed one is skipped *and* forgotten while its neighbour is kept, storing
twice keeps the latest, and the automatic half is off by default.

**Gates**: the wheel, 11/11 checks, the full suite, pytest, the strict HTML build.

<a id="rg3-24"></a>
### RG3.24 — the generator API stops saying "Latex" (2026-09-27, #2681)

**The declaration API is what every definition file is written in**, and it said LaTeX while it wrote
Markdown - since revision2026b step RG3.14, when the LaTeX and RST branches went. 184 renamed
occurrences, and the diff is almost all in `definitions/`:

| was | is | uses |
|---|---|---|
| `DefLatexDataAccess` | `DefDataAccess` | 66 |
| `DefLatexStartTable`, `DefLatexStartTable3` | `DefStartTable`, `DefStartTable3` | 34 |
| `DefLatexFinishTable` | `DefFinishTable` | 27 |
| `DefLatexOperator` | `DefOperator` | 17 |
| `DefLatexStartClass` | `DefStartClass` | 13 |
| `PyLatexRST` | `DeclarationWriter` | 18 |
| `GetTypesStringLatex` | `GetTypesStringDocu` | 9 |
| `Latex2RSTlabel` | `MarkdownLabelName` | 5 |

**`pybindTypes.declarationCalls` lists those methods by their names, as strings**, and it is what the
pybind emitter replays a recorded declaration run through - so a method renamed in the class and not
in that list is a silent failure of the whole emitter. The textual rename covered both, which is the
one thing about this diff that was not mechanical.

**The local names went with the class**: `plr` - the abbreviation of `PyLatexRST` - and its seven
relatives are `writer` and its relatives, 106 occurrences, and the emitters' own
`sLatexObjectClass`, `sLatexGlobalNames`, `sLatexGlobalItemIntros`, `sLatexIntro`, `latexIntro`,
`moduleNameLatex`, `pyNameLatex` and `ReplaceDefaultArgsLatex` now say what they hold, which is
Markdown or the text of a page.

**What is named after real LaTeX keeps its name**, which was the rule the step set: `Str2Doxygen`
escapes for a C++ comment, `latexToMarkdown.py` is named after what it converts **from**,
`LatexText2Markdown` after what it is given, `ToLatex` escapes `{`, `}`, `_` and `&` for text that
goes into that converter, and `CleanLatex` and `RemoveLatexCommands` strip real LaTeX out of one.

**Four dead things were found by reading every occurrence**, which is the part of a rename that pays
for itself: `ParameterChanges2LatexRST` (19 lines, a `\bi ... \item ... \ei` list, no caller since
the RST output went), 33 lines of commented-out LaTeX tables in the two docs emitters - `longtable`,
`\hline`, and assignments to a `.sLatex` attribute the writer class has not had since RG3.14, so they
could never be revived - three locals that were assigned and never read, and
`latexSymbol = latexSymbol`.

**One rename was wrong for a minute and the generator said so**: `sPythonGlobalNames` looked dead
next to the three locals that were, and `NameError: name 'sPythonGlobalNames' is not defined` came out
of the regeneration 20 lines later. That is the gate working, and it is why the gate was chosen.

**The gate: the regeneration is a no-op.** 184 renamed occurrences, 55 deleted lines, and **not one
byte** of `src/Autogenerated/`, `docs/generated/` or `tools/generators/generated/` differs - run after
every one of the three passes. A rename that changes output is not a rename.

**And one stale name in the documentation**, found by the same search: `definitions/README.md` told a
writer that a `## Title` goes inside `latexText`. There is no such field anywhere in the repository -
the field is **`sectionText`**, which is in the table five lines above it.

**What is NOT renamed, and goes to the maintainer as RG3.24.4**: the `latexSymbol` family -
`latexSymbol` (12 uses in four emitters), `ExtractLatexSymbol`, `stringLatexSymbol`, `addLatexSign`.
The step reserved this one, because the thing it names **is** LaTeX: the `$...$` a parameter
description opens with, which becomes mathematics on the page. The options are in the plan; the
recommendation is `mathSymbol`, because the reader of the emitter wants to know *what* it is and the
markup is the converter's business.

<a id="rg3-22"></a>
### RG3.22 — the settings sections say how to look a setting up (2026-09-27, #2659)

Two places introduce a settings structure to somebody who has not written a model yet, and both told
them only how to *assign* a value: the *Simulation settings* section of Exudyn Basics and the
*Visualization settings dialog* section of the GUI chapter. The answer to *what is this setting
called* is `python -m exudyn dialogs sim` and `... dialogs vis`, which open the tree the renderer's
**V** key opens with no model and no renderer - and the second section even explained that **V** does
not work without a running render loop, without saying what does.

Two sentences and a line of code in the first, one sentence in the second, both pointing at
{ref}`sec-commandline`, where the command and what closing the dialog prints are described. Named with
them: **CTRL-F** to find a setting and *copy line* to take the statement that sets it, because that is
what makes the browsing useful rather than merely possible.

**Gates**: the strict HTML build, and the two references resolve.

<a id="rg3-22"></a>
### RG3.22 — the settings sections say how to look a setting up (2026-09-27, #2659)

Two places introduce a settings structure to somebody who has not written a model yet, and both told
them only how to *assign* a value: the *Simulation settings* section of Exudyn Basics and the
*Visualization settings dialog* section of the GUI chapter. The answer to *what is this setting
called* is `python -m exudyn dialogs sim` and `... dialogs vis`, which open the tree the renderer's
**V** key opens with no model and no renderer - and the second section even explained that **V** does
not work without a running render loop, without saying what does.

Two sentences and a line of code in the first, one sentence in the second, both pointing at
{ref}`sec-commandline`, where the command and what closing the dialog prints are described. Named with
them: **CTRL-F** to find a setting and *copy line* to take the statement that sets it, because that is
what makes the browsing useful rather than merely possible.

**Gates**: the strict HTML build, and the two references resolve.

<a id="rg3-24-4"></a>
### RG3.24.4 — the latexSymbol family is mathSymbol (2026-09-27, #2699)

The maintainer chose option B: `latexSymbol` → `mathSymbol` (9), `ExtractLatexSymbol` →
`ExtractMathSymbol` (9), `stringLatexSymbol` → `mathSymbolString` (7), `addLatexSign` → `mathSign`
(3), in `itemModel` and the four item emitters. The comment above the function now says the rule
option C wanted in its name - only a `$...$` at the very **start** of a description is split off - so
the half of C that cost nothing came along.

**Gate**: the regeneration wrote byte-identical files; the only generated file that moved was the
tracker log, because the issue was raised in the same run. With this the generator API has no name
left that says LaTeX where it does not mean it.

<a id="rg2-3-1"></a>
### RG2.3.1 — the scene as data, and the text export removed (2026-09-27, #2700)

The maintainer's decision on the proposal of the same day: *"yes do that and totally remove the .txt
graphics export"*.

**`SC.renderer.GetGraphicsData()`** returns the five primitive lists of the graphics data - lines,
spheres, circles, texts, triangles - as numpy arrays, each with an `items` array of shape (n,3):
system number, `exudyn.ItemType` value, item index. The encoded `itemID` is decoded in C++ with
`ItemID2IndexType`; the negative IDs of static scene elements, which that function does not decode,
come out as `[-1, 0, code]` so the kind stays visible. It is built the way
`RedrawAndGetImage(True)` builds its data - post-processing data of every system, then
`UpdateGraphicsDataNow()` and `UpdateGraphicsData()` - so **it needs no window**, and the tests
confirm `IsActive()` is False throughout.

**One thing in it is there for a running renderer**: the GIL is released while the graphics data
locks are taken, because a render thread can hold such a lock while it waits for the GIL in a
graphics user function, and a Python thread spinning on the lock with the GIL held would wait
forever.

**The first measurement of the pendulum** (a ground with a checkerboard, a brick, a revolute joint)
is what the suite will compare: 200 triangles for the ground, **12 for the brick** - six faces of two
- and 192 for the joint, 12 lines, and with `nodes.showNumbers` one text, `N0`.

**Removed**: the `TXT` branch of `GlfwRenderer::SaveSceneToFile` and of the file ending (106 lines),
the four `exportImages.saveImageAsText...` settings, and `exudyn.plot.LoadImage`. An unknown
`saveImageFormat` - `TXT` included now - says `exportImages.saveImageFormat='TXT' is not a format; use
'PNG' or 'TGA'` instead of a generic *illegal format*. `parameterConversionTestReference.txt` lost the
four names from its `exportImages` row, rewritten with `recordReference`.

**`PlotImage` stays and draws `GetGraphicsData()`** - it is the one thing the export was for, a
vector figure of a model for a paper - with `trianglesAsLines` and `circleSegments` taking over what
`LoadImage` and `general.circleTiling` did. The two NGsolve examples that used the export do it that
way now; both are in `if False` blocks.

**Found on the way and raised, not fixed** (#2701, RG2.3.2): `PlotImage(plot3D=True)` raises
`TypeError` with every matplotlib since 3.6 - `fig.gca(projection='3d')` - so both examples'
3D figures had been broken for years behind their `if False`.

**Seven tests** (`test_graphicsData.py`), none opening a window: the shapes of every array, the brick's
12 triangles under its own item, the counts identical on a second call, a setting that hides the
bodies removing their triangles and `showNumbers` adding `N0`, the brick's triangles **moving** after
half a second of simulation while their number stays, the export and `LoadImage` gone, and
`PlotImage` writing a PDF.

**Gates**: the wheel, 11/11 checks (TIER 1 drift, which is the new function and the four removed
settings), the full suite, pytest, the strict HTML build.

<a id="rg3-25"></a>
### RG3.25 — the TAB that ate a backslash (2026-09-27, #2683)

Three descriptions in `definitions/pybindSymbolic.py` - the Real, Vector and Matrix introductions -
read `turing on recording by using <TAB>exttt{exudyn.symbolic.SetRecording(True)}`. Somebody wrote
`\texttt` in a literal that was not raw, Python made the backslash-t a TAB, and the TAB survived the
later conversion of the literal to `r"""..."""`. They are
`` `exudyn.symbolic.SetRecording(True)` `` now, the three `turing` are `turning`, and `veryfy` in the
paragraph above them is `verify`.

**The check is the part worth keeping.** `CheckNoLatex` finds a LaTeX command by its backslash, and
this defect is exactly a LaTeX command without one. `CheckTabs` rejects any TAB in a description -
a TAB means nothing in Markdown - and exempts what is not prose: a value passed to one of
`CODE_KEYWORDS`, and the argument of `pb.CppCode(...)`, which indents generated C++ with TABs in
`pybindEnums.py`.

**Measured before it was switched on**: every literal TAB in `definitions/` is one of the three
defects, the two `CppCode` indents, or inside commented-out C++ in `pybindSymbolic.py`, which is a
Python comment and not a string. So the check fired on exactly the three, before the fix, and on
nothing after it. Four tests (`test_checkDefinitionsTabs.py`): a TAB in a description is found **on
its own line** rather than the line the literal starts on, the two exemptions, and the definitions
have none.

**Gates**: the wheel, 11/11 checks, the full suite, pytest, the strict HTML build; the regeneration
changed the three paragraphs of `Symbolic.md` and nothing else.

<a id="rg3-26-1"></a>
### RG3.26.1 — the PDF gets its missing chapter back (2026-09-27, #2702)

**The drift has a date.** RG3.15 (2026-09-25) carried out the maintainer's table of contents in
`index.md`: a new chapter *Performance, errors and solver failures*, and the command line and the
results monitor moved from chapters of their own into *Advanced topics*. `pdfIndex.md` was not
touched, so the PDF built since then had **no performance chapter at all**, listed the command line
and the results monitor as chapters, and put *Advanced topics* before the *Tutorial*. Nothing failed,
because every page was still in *some* toctree.

`pdfIndex.md` takes the user-manual toctree of `index.md` without `README`. Measured afterwards with
the same script as for #2697: **22 shared entries in the same order**; only in `index.md` are
`README`, the examples index and the test-models index - the three differences the PDF means to have.
Nothing is only in `pdfIndex.md`.

**Checked in the PDF itself**, from the LaTeX source of `exudev docs --pdf`: `\chapter{Performance,
errors and solver failures}` is there once, between *Renderer, graphics and visualization* and
*Advanced topics*, and the command line and the results monitor are sections of *Advanced topics*, as
in the HTML.

**What is not done, deliberately**: how the two files are kept from drifting again - generate one
(A), check them against a declared difference (B), or say so at the top (C) - is the maintainer's
choice and stays #2697. This step only removed the drift that had already happened.

**Gates**: the HTML and the PDF build (xelatex), 11/11 checks, the full suite, pytest.

<a id="rg3-13-1"></a>
### RG3.13.1 — the plan references in the code are gone (2026-09-27, #2649)

**The count had grown.** RG3.13 left 235; measured again, `revision2026` stood **330 times in 177
files** of `src/`, `tools/`, `python/` and `definitions/` - the issue records and the generated files
excluded, where citing a step is the convention or the text is not written by hand. The work since
RG3.13, this session's included, had kept writing `(revision2026b step RG12.23)` into comments. It is
**0** now.

**Three passes, and the lesson of RG3.13 decided their shape**: its third pass let a pattern cross a
line break and merged prose into code. So nothing here touches a line of code:

| pass | what | how many |
|---|---|---|
| 1 | a single line of a comment or a docstring, found with `tokenize` for Python and a small scanner for C++: a parenthetical keeps its issues and loses the step, a comment that was **only** a reference keeps its issue, a trailing clause goes | 202 in 142 files |
| 2 | a reference wrapped across two **full-line** comments: the two lines are joined, the same rules applied, and wrapped again to their width - never a block, because joining a block merged two separate comments and a banner line in the first attempt, which was reverted | 37 pairs |
| 3 | by hand, a sentence each, where the reference was part of what the sentence said | 85 edits |

**What the hand pass decided**, as a rule: what a sentence said about the **history** of the code
goes, what it says about the **code** stays, and an issue replaces a step where there is one -
*"Until revision2026b step RG12.6 these were 325, 188 and 113 pixels, which is what they still are at
the default width"* is *"At the default width of 1024 they are 325, 188 and 113 pixels (#2667)"*.
Four references were to facts of the info document (`revision2026 fact 14`, `19`, `21`) or to a
decision (`D14`); they went, because the sentence around them is true without them. Three printed
messages changed with them: the note of `runTestSuite.py` on a known Windows/Linux difference names
`UnresolvedOnLinux()` instead of *revision plan phase R10*, `regenerate.py` no longer cites a fact, and
one reason string of `checkExtras.py`.

**The proof that no code changed**, run over all 175 changed files: the Python AST with the docstrings
blanked is identical before and after, and so are the C++ tokens with the comments removed. The
exceptions were the strings named above - and two C++ files whose comments contain a quote, which the
comment stripper does not remove and which were read by eye. Three comment lines that had been
nothing but a reference had become blank lines inside comment blocks; they are removed.

**What the measure did not see** (#2703, RG3.13.2): **88 bare step numbers in 39 files** -
`step R6.3.8`, `the rule RG6.2.11 wrote down` - the same rule without the plan's name. RG3.13.1 counted
the word `revision2026`, which is what RG3.13 had counted.

**Gates**: the wheel, 11/11 checks (TIER 1 drift: a comment in `symbolic.pyi`), the full suite,
pytest, the strict HTML build. Four generated pages of the Python utilities changed, because their
docstrings carried the references and are published.

<a id="rg3-26"></a>
### RG3.26 — the two tables of contents are held together by a check (2026-09-27, #2697)

The maintainer chose **option B**: both files stay hand-written, and `tools/checkTocs.py` compares
the entries of their `{toctree}` blocks. What may differ is declared in the tool - `README`, the
examples index and the test-models index are HTML only, nothing is PDF only - and **a declared entry
that is no longer there is a finding too**, so the declaration cannot go stale while the files move on.

**It finds what it was written for**: run against `pdfIndex.md` as it was before RG3.26.1, it reports
the missing `performanceErrors`, the two pages listed as chapters only in the PDF, and *the order
differs at entry 4* - the whole drift of #2702 in four lines. On the repository it reports nothing.

It runs in `exudev generate --all-checks` beside `checkHeadings`, so the next page added to one file
and not the other fails the gate of that commit. Both files say so in an HTML comment above their
first toctree, which is where somebody adding a page is looking. Four tests
(`test_checkTocs.py`), the first of them the drift that happened.

**Gates**: 12/12 checks, the full suite, pytest, the strict HTML build.

<a id="rg2-3-2"></a>
### RG2.3.2 — PlotImage draws in 3D again (2026-09-27, #2701)

Four lines of `exudyn.plot.PlotImage`, the three the issue named and one more found by the test:

- `fig.gca(projection='3d')` is `fig.add_subplot(projection='3d')`; the keyword was removed in
  matplotlib 3.6, so the 3D mode - the only one that draws triangles - raised `TypeError`;
- the 2D branch translated y and z by the **x** component of `HT`, and now by their own;
- the 3D branch ended every line segment at `z[j]`, and now at `z[j+1]`;
- **`azim` and `elev` were documented and ignored**: `view_init(elev=0., azim=0.)` was hard-coded.
  The test asked for 30 and 20 degrees and got 0 and 0.

**And one outside the four**: `PlotImage` called `plt.show()` on the `Agg` backend, because it
compared `get_backend()` with `'agg'` while matplotlib says `'Agg'` - the *FigureCanvasAgg is
non-interactive* warning of the RG2.3.1 tests. The comparison is case-insensitive now.

**Two tests** in `test_graphicsData.py`: the 3D mode writes a PDF and the axes have the angles that
were asked for, and a translation of `HT` by `[0, 5, 0]` moves the drawn y by 5 and x by nothing - the
slip that no default `HT` could show.

**Gates**: the wheel, 12/12 checks, the full suite, pytest, the strict HTML build.

<a id="rg3-13-2"></a>
### RG3.13.2 — the bare step numbers (2026-09-27, #2703)

**87 of the 88 are gone**, in 38 files, by hand: 82 edits, one of them the same comment six times in
`PyMatrixVector.h`. Every step number was looked up in the two plans for its issue, and where the
plan names one the comment cites it - *"the rule RG6.2.11 wrote down"* is *"the rule of #2608"*,
*"step R6.8"* is *#2538*. **The one that stays** is `issueStore.py`'s description of the `planStep`
field, whose value IS a step number: it reads *"the step of the revision plan, e.g. "R5.4.5""*.

**A dozen of them were not only history but wrong.** Comments that promised a future the plan had
already delivered: *"the LaTeX string is still built, and dies with the LaTeX branch in R7.1.7"*,
*"the figures of an item live with the LaTeX chapters until R7.1.7 moves them"*, *"the LaTeX and RST
branches above go in R7.1.7"*, and a docstring naming *"the LaTeX and RST twin below"* of a function
that has had no twin since RG3.24. They say what is there now. Six comments in `PyMatrixVector.h`
said *"passing it on is step R6.3.8"* where the code does not pass it on; they say that.

**The same proof as RG3.13.1**: the Python AST without docstrings and the C++ tokens without comments
are unchanged in all 39 files; one C++ file was read by eye because its comment holds a quote.

**Gates**: the wheel, 12/12 checks, the full suite, pytest, the strict HTML build.

<a id="rg12-10-1"></a>
### RG12.10.1 — the import note is one short line (2026-09-27, #2705)

The maintainer, on what `import exudyn` printed with a settings file of eight visualization
settings:

```
NOTE: 0 setting(s) from %USERPROFILE%\.exudyn\config.json, and 8 visualizationSettings for every such
structure that is created (exudyn.misc.overrideSettings.Print() for the list; EXUDYN_NO_USER_SETTINGS=1
to ignore them)
```

*"this is too long"*. It is now

```
NOTE: 8 visualizationSettings read from ~/.exudyn/config.json
```

- a count that is 0 is not printed (*"2 config settings and 8 visualizationSettings"* when both are
  there);
- the file name is `overrideSettings.ShownFileName()`: the home directory as `~`, forward slashes -
  the same privacy rule `python -m exudyn info` follows since #2695;
- the pointers to `Print()` and to `EXUDYN_NO_USER_SETTINGS` are in the documentation, not in the
  import output.

**`"suppressOverrideSettingsWarning": true`** in the file switches the note off. It is a flag of the
file beside `version`, not a section: `overrideSettings.flagNames` lists it, so `Load()` does not call
it an unknown section, and it stays in the store, so `Store(...)` does not drop it when it rewrites the
file. It does not exist unless a user writes it. **It suppresses the note only**: a setting that could
not be applied is still reported - that is a defect, not information - and those warnings now name the
file too.

Three tests, the first two in a fresh interpreter because the note is printed by `import exudyn`: a
file of two visualization settings gives exactly one line, with no `config settings` and no
`Print()` in it; the flag gives no line at all and no *unknown section*; the shown name of the default
file is `~/.exudyn/config.json`. The documentation of the override settings shows the new note and the
flag.

<a id="rg2-3-4"></a>
### RG2.3.4 — PlotImage in 3D shows the triangles (2026-09-27, #2706)

Reported by the maintainer with `serialRobotKinematicTree.py`: the line drawing came out as a PDF,
and with `trianglesAsLines=False` the 3D model was not there - *"a zoom problem or so; also check the
coordinates stored in triangles"*.

**The coordinates were right**, measured: 2428 triangles of two items spanning -0.39 to 0.69 m,
global and current. **The limits were not**: ±1.7 mm. The only lines of the model are the bases of one
marker and two sensors, ±1.5 mm around the origin, and `ax.autoscale()` of a 3D axes takes lines into
account and not a `Poly3DCollection`. A second call, `plt.autoscale()` after the 3D branch, would have
undone any limits set there.

The 3D branch now sets its limits from **every point drawn**, lines and triangles, as a cube when
`axesEqual` (so the box aspect of 1:1:1 is to scale); `plt.autoscale()` runs for 2D only. The robot is
drawn, at ±0.44 m around its centre. One test with a triangle of 2 m and a line of 2 mm: the limits hold
both, and x and y have the same range.

<a id="rg3-3-2"></a>
### RG3.3.2 — the logo once on the first page of the PDF (2026-09-27, #2707)

The first page of the PDF carried `ExudynLOGO1.9.jpg`, and then `intro2.jpg` - which is **two pictures
in one file**, the gear logo generated with ChatGPT above the piston engine. The maintainer: *"remove the
first one and make the second as wide as the piston engine image"*. Two pictures in one file cannot have
one width, so the file was cut into `titleLogo.jpg` (1024 × 664) and `titleEngine.jpg` (1986 × 1129)
along the white band between them, found by measurement, and `pdfIndex.md` shows both at 370 px - the
width the engine had inside `intro2.jpg`. Checked in the built PDF.

`intro2.jpg` is referenced by nothing now. It stays until the maintainer says whether it goes: deleting
a tracked file is theirs to decide. `ExudynLOGO1.9.jpg` stays - `README.rst` uses it.

<a id="rg3-27"></a>
### RG3.27 — the mass-spring-damper tutorial comes first (2026-09-27, #2708)

The maintainer: *"The mass-spring-damper tutorial shall be the first tutorial in the docs, not the
last one."* It was the one section of `docs/manual/tutorial.md` itself, and the toctree of the other
four tutorials stands at the top of that page - which it has to, since #2646, or they nest under the
section - so it came after all four, as 4.5 of the PDF. It is a page of its own now,
`tutorialSpringDamper.md` with the target `sec-tutorial-springdamper`, **first** in the toctree; the
text is moved unchanged, and its heading is the page title.

With it, on the maintainer's answer to RG3.3.2: `docs/figures/intro2.jpg` is deleted - the two
pictures it held are `titleLogo.jpg` and `titleEngine.jpg` (#2707).

<a id="rg2-3-5"></a>
### RG2.3.5 — PlotImage saves into the output directory (2026-09-27, #2711)

The maintainer: *"the PlotImage should also get the output directory added."* `PlotSensor` has sent
its saved figure through `OutputFilePath` since #2454; `PlotImage` wrote the name as given, so the
maintainer's `serialRobotKinematicTree.py` wrote `solution/...` into `python/Examples/` even under a
runner that sets the output directory. One line, the docstring says where the file goes, and a test
that sets `exudyn.config.outputDirectory` to a temporary folder and finds the PDF there.

<a id="rg10-1"></a>
### RG10.1 — exudev scripts: what an old script has to change (2026-09-27, #2712)

Asked for by the maintainer for teaching next week: *"a command-line exudev tool for now, only a static
checker of folders, looking at all .py files where exudyn is imported"*. `exudev scripts <folder>`
runs `tools/checkUserScripts.py`, which **parses and never runs**, and says per file and line:

| finding | where the list comes from |
|---|---|
| a name a star import from exudyn no longer provides - `np`, `sin`, `graphics`, 12 names - with the import line | measured on the sources before `__all__` (#2444), statically: the names every module of the `exudyn.utilities` chain had imported for itself - exactly the 12 of the API change table |
| a removed name with its replacement: the 11 vector helpers, the 24 `GraphicsData...` aliases, `LoadImage` | the removal commits (#2442, #2443) and #2700 |
| a deprecated function or setting, with what to use instead | **read from `definitions/` each time**: 22 functions marked DEPRECATED in the pybind definitions, 94 deprecated settings with the path that replaces them |
| a removed setting or argument: `saveImageAsText...`, `openVR`, `fontScalingMacOS`, `rBoundingSphere` | the steps that removed them |
| `exu.robotics` and the other submodules `import exudyn` does not load | asked of the installed package |

**Two things the first run got wrong, and why they matter for a teacher's folder**:

- a function parameter called `np` made `np` look imported for the whole file. Names are now resolved
  **per scope** - module, function, class, lambda - so a parameter binds only its function;
- `window.renderWindowSize` was reported 109 times in the repository, because `window` is a deprecated
  member at the top of the visualization settings **and** the current member of every view. A
  deprecated setting now also names the structure it sits in, and `view0.window` is not a finding.

**Run over the repository's own 342 scripts: 54 findings in 17 files, in 1.5 s.** Four are real breaks
in scripts that no suite runs - three examples call `AddEdgesAndSmoothenNormals` without `graphics.`,
one uses `graphics` without importing it - and the rest are deprecated forms. That is RG10.1.2 (#2714);
running the scripts, after a check for paths that do not travel, is RG10.1.1 (#2713).

A script that does not parse - Python 2 is the usual case in an old folder - is reported as such and
not checked; a `.py` file that does not import exudyn is counted and skipped. `--check` fails if
anything was found, and the paths are printed relative to where `exudev` was started. Nine tests
(`test_checkUserScripts.py`), among them the two false findings above as cases that must stay silent.

**Also in this commit, on the maintainer's word**: RG6.3 (#2583) closed as superseded; RG10.10 moved
to where its number belongs; RG12.5.2 given the measurement it lacked; RG13 created.

<a id="rg10-1-2"></a>
### RG10.1.2 — the repository's scripts take what exudev scripts says (2026-09-27, #2714)

The maintainer asked for the four real breaks to be fixed and the step done. **92 lines in 18 files**,
and `exudev scripts` over the 342 examples, test models and mini examples now reports **0**.

**The checker learned two things first**, both found on these files:

- `SC.WaitForRenderEngineStopFlag()` and `mbs.WaitForUserToContinue()` warn in C++, but their
  descriptions in `definitions/` do not start with DEPRECATED, so the first version did not know them.
  It now also reads the `renderer.DeprecationWarning("old", "new")` calls of
  `MainSystemContainer.cpp`: 19 more findings, all in scripts that also used the renderer the old way.
- `openGL.light0ambient` is deprecated **to a dummy** - it has done nothing since 1.10.80 - and the
  checker said *"use openGL.dummy"*. It says *"it has no effect; remove it"*, and the line is removed.

**The four breaks**: `NGsolveGeometry.py`, `humanRobotInteraction.py` and `stlFileImport.py` call
`graphics.AddEdgesAndSmoothenNormals` now, and `nMassOscillatorEigenmodes.py` imports
`exudyn.graphics`. **`NGsolveGeometry.py` was a known failure of the examples runner**, recorded as
*"fails inside the NGsolve geometry construction under exec(...)"* - the failure was this `NameError`.
It passes, and the runner reported its own exclusion as dead, so the exclusion is removed.

**The rest** are the current forms: `SC.renderer.Start()`, `Stop()` and `DoIdleTasks()` (with `sc` and
`env.SC` where the script calls its container that), `mbs.SolveStatic/SolveDynamic`, the relocated
settings under `view0.scene`, `view0.window`, `view0.camera` and `openGL.light0`, and
`SC.renderer.GetState/SetState`. In `simulatorCouplingTwoMbs.py` the `SC.renderer.Detach()` before a
second container is created goes: creating a container attaches it to the renderer.

**Checked**: the full test suite (both changed test models are in it) and the examples runner - 168 of
170 pass. `rendererNOGLFWexample.py` is the one known failure; `NGsolveOCCboundaries2.py`, which
this step did not touch, failed once with netgen's *"Could not allocate localheap, heapsize =
3200000000"* while the runner ran examples in parallel, and runs through when run alone.

**With it**: the stray `coordinatesSolution.txt` in the repository root - committed by mistake - is no
longer tracked, and `.gitignore` ignores `coordinatesSolution.txt` and `.sol` wherever they are
written: the default solution file name of the simulation settings has no directory, so a model run
from any folder writes it there.

<a id="rg12-24"></a>
### RG12.24 — the files a run writes by default go into solution/ (2026-09-27, #2718)

The maintainer, on the stray `coordinatesSolution.txt` files: *"This is the problem that still examples
write into the root instead of solution"* - and, once the cause was named, *"as we anyway made some
small API changes, this would be the right time to resolve"*.

**The cause** is one default: `solutionSettings.coordinatesSolutionFileName` was `coordinatesSolution`,
without a directory, so every model that did not name its solution file wrote it beside itself. It is
`solution/coordinatesSolution` now, and **`solverInformationFileName` and `restartFileName` go with
it**, to `solution/solverInformation.txt` and `solution/restartFile.txt`. The directory is created when
the file is written, as before.

**Measured before the change**, in the output folders the two runners give each model: the examples
suite wrote **91 files beside the script, every one of them the default solution file** (86 `.txt`, 5
`.sol`) - no sensor file among them. **After it, both suites write nothing beside a script**: the
top level of all 141 example output folders and of all test model output folders is empty, and
everything is under `solution/`.

**What a script has to change, and how it is found.** `exudev scripts` learned two findings:

- a file the script **names without a directory** - `coordinatesSolutionFileName`,
  `solverInformationFileName`, `restartFileName`, `saveImageFileName`, a sensor's `fileName=`, the
  `resultsFile=` of the processing functions - including a computed name whose first piece has no
  directory (`'info' + str(i) + '.txt'`);
- the **default solution file read back by its old name** - `LoadSolutionFile('coordinatesSolution.txt')`,
  `np.loadtxt(OutputFilePath('coordinatesSolution.txt'))`, a `PlotSensor` of it - unless the script
  writes that name itself.

Over the repository: **22 findings in 19 files**, all fixed. Twelve examples and two test models read
the default solution file by its old name, which **the new default would have broken** - that is what
the maintainer's *"needs to be consistently changed in a script then"* meant, and why the check came
before the change. Seven scripts set a solution file name without a directory, and one publication
example wrote two sensor files beside itself and read them back. The processing functions have no
default file name, and every `resultsFile` in the repository already names a directory.

**For a user**: `revisions.md` says what changed and what `exudev scripts` finds; the tutorials, the
getting-started example, the GUI chapter and the `SolutionViewer` docstring read
`solution/coordinatesSolution.txt`. **The other way** the maintainer named - setting the output
directory once in Spyder - is in the documentation of the environment variables of the Exudyn
module: the *Run code* startup line of the IPython console, and `setx` outside Spyder.

Two more tests of the checker (13). **Gates**: the wheel, 12/12 checks, the full suite, the examples
(169 of 170; the known `rendererNOGLFWexample.py`), pytest, the strict HTML build.

<a id="rg2-3-3-1"></a>
### RG2.3.3.1 — the graphics regression test runs: the machinery and every graphics function (2026-09-27, #2704)

The first sub-step of the graphics test, built so that whether the approach carries is seen now.

**The machinery** is `python/testing/graphicsRegression.py`: `Fingerprint(SC.renderer.GetGraphicsData())`
reduces a scene to, per item and per kind of element, the exact **count** and **min, max and mean** of
points, colors, radii and normals, plus the texts; `Differences(reference, current)` compares two of
them and says what differs in one line each - *"Object 12: triangles 256 -> 128"*,
*"Object 7: triangles points mean[2] 0.0 -> 0.5"*. The maintainer's decisions are in it: a relative
tolerance of **1e-5** (as `|a-b| <= 1e-5 (1 + max(|a|,|b|))`, so that a mean near zero is not held to a
relative tolerance of nothing), **per item up to 32 items and per item type above**, and the
references in **`python/testing/graphicsReferences/`**, one JSON file per case. A missing reference,
or every one when `EXUDYN_RECORD_GRAPHICS_REFERENCES=1` is set, is written and the test **fails once
saying so**, so that nothing is recorded without being looked at.

**The first case is every function of `exudyn.graphics`** - 32 of them, each on a ground object of its
own and at a place of its own, so that the object index in a difference names the function; the test
translates it back (*"Torus - Object 12: ..."*). Sphere (with and without edges), Lines, Circle, Text,
Cuboid, BrickXYZ, Brick (and rounded), Cylinder (and hollow, half), Tube, Torus, RigidLink,
SolidOfRevolution, Arrow, Basis, Frame, Quad, CheckerBoard, SolidExtrusion, LinkedCylinders,
InvoluteGear, ToothedRack, BallBearingRings, and the transforms Move, Transform, MergeTriangleLists,
InvertTriangles, AddEdgesAndSmoothenNormals, FromPointsAndTrigs and an STL written and read back.

**What it measured, and what that says about the approach**:

- **the reference is 26 KB for 32 items and 5,700 triangles**, and the run takes **0.7 s** with the
  import - so a case per item family, as RG2.3.3.5 plans, stays small;
- the counts are what the functions promise - a brick is 12 triangles, BrickXYZ with edges adds 12
  lines, the gear is 2392 triangles - and **two recording runs in a row agree exactly**, so the data is
  deterministic within a machine; across compilers is what the tolerance is for, and the Linux CI will
  be the first measurement of it;
- **32 is exactly the limit**: a 33rd function would switch the fingerprint to per item type. The test
  asserts that it is per item and holds one item per function, so that happens loudly, and a second
  case is the answer then.

**Three things the functions do that a test has to know**: `BallBearingRings` returns a **dict** of
three graphics, not a list; `InvertTriangles` refuses a triangle list without normals; `FromSTLfile`
needs the optional numpy-stl, so the test uses `FromSTLfileASCII`. And a second test checks the
comparison itself: a changed text and a moved body are each reported.

<a id="rg13-1"></a>
### RG13.1 — the state of the documentation of every item, measured (2026-09-27, #2715)

`tools/itemDocumentationReport.py` reads the 97 item definitions and the 342 scripts of the repository
and writes the table, [itemDocumentationState.md](itemDocumentationState.md): per item, the words of the
class description and of the equations text and how many sections it has, whether the page shows a
figure, which parameters and output variables lack a description, whether there is a MiniExample, and
in how many examples and test models it is used - the same search as the links on its page. It can be
run again at any time, which is how the progress of RG13 will be measured.

**The summary, by kind of item**:

| kind | items | no equations text | no figure | no MiniExample |
|---|---|---|---|---|
| Node | 16 | 9 | 16 | 16 |
| Object (Body) | 7 | 0 | 6 | 2 |
| Object (SuperElement) | 4 | 0 | 3 | 2 |
| Object (FiniteElement) | 7 | 1 | 7 | 4 |
| Object (Connector) | 19 | 1 | 14 | 12 |
| Object (Constraint) | 3 | 0 | 3 | 1 |
| Object (Joint) | 10 | 1 | 5 | 9 |
| Object (Object) | 1 | 0 | 1 | 0 |
| Marker | 18 | 10 | 17 | 17 |
| Load | 4 | 0 | 4 | 3 |
| Sensor | 8 | 7 | 8 | 8 |
| **all** | **97** | **29** | **84** | **74** |

**What it says**:

- **The parameters are described**: 3 of 985 have fewer than three words (`name`, the same in every
  item, is not counted), and every one of the 413 output variables has a description. RG13 is not about
  the tables of a page.
- **The objects have their equations; nodes, markers and sensors mostly do not**: 26 of the 29 items
  without an equations text are nodes (9), markers (10) and sensors (7) - the items whose behaviour is
  least obvious to a new user are the objects, and those are written.
- **A figure is the rare case**: 13 of 97 items show one, and most of them are joints and connectors.
- **A MiniExample is the rarer case**: 23 of 97 have one - the gap RG13 was created for, and the one the
  graphics test (RG2.3.3.5) and the figures of the items both depend on.
- **7 items are used in no example or test model**: NodePointSlope1, NodePointSlope12, NodeGenericAE,
  ObjectANCFCable, ObjectANCFThinPlate, ObjectContactSphereTorus, MarkerNodeODE1Coordinate. The search
  is the one of the documentation pages - `mbs.Add<Type>(<Name>(` and the item's `Create...` function -
  so an item created only through a helper such as the beam utilities is counted as unused; ANCFCable is
  probably such a case. It is where a MiniExample adds the most.

**What the table cannot say** is whether a description is **right** - that is RG13.3, the one-time
synchronization with the implementation. What it can say is what is missing, per item, and that is
what RG13.2 needs to decide what the ideal page contains.

<a id="rg12-25"></a>
### RG12.25 — store positions stores every open window (2026-09-27, #2719)

The maintainer tested 1.12.128 with the solution viewer, the settings dialog, the render window and two
sensor plots open, pressed **store positions**, and got one line: the settings dialog. The button
stored *"where this dialog is"* - which is what it said, and not what a user means by it.

It lists and stores now:

- **this dialog**, as before;
- **the other interactive dialogs** - `exudyn.interactive.openDialogs`, a list every
  `InteractiveDialog` joins when it opens and leaves when it closes, so the SolutionViewer is stored
  while it is open and not only when it closes with `storeDialogPositions` on;
- **the PlotSensor windows**, through the new `exudyn.plot.PlotWindowGeometries()`, under the names
  their sequence gives them;
- **the render window where it is**: the render state's `currentWindowSize` and
  `currentWindowPosition`, not what `view0.window` says - the difference the maintainer found. They
  go into the `visualizationSettings` section of the file as `view0.window.renderWindowSize` and
  `renderWindowPosition`, merged with what is there, **and into the dialog**, through the same path an
  edit takes, so that the settings on screen, a later *store settings* and the file agree.

The window that opens before anything is written lists all of them. A test presses the button on a
settings dialog in a withdrawn Tk root, with a stub viewer and a stub plot window, and finds both in
the file under their names; the render window part needs a running renderer and is for the
maintainer's hands.

<a id="rg12-26"></a>
### RG12.26 — the SolutionViewer follows its window, and can be given a size (2026-09-27, #2720)

Three things the maintainer found, in `InteractiveDialog`, which the SolutionViewer, the mode shapes
and interactive simulations all are:

- **the sliders did not follow the width**: every widget was placed sticky, but no column had a
  weight, so a wider window added empty space on the right. The columns right of the first - the
  sliders - take the width now, the first column keeps the width of its labels, and the Run button
  spans all columns;
- **the size could not be given**: `SolutionViewer(..., windowSize=[w, h])` and the same argument of
  `InteractiveDialog`. A size given in the script wins over a stored one, as it does for the render
  window. Stored it is by store positions (RG12.25) or when it closes with `storeDialogPositions` on;
- **"t = 1.0" above the Run button**: the label shows the time of the dialog's own time integration,
  which the viewer does not run, so it said 1.0 whatever row was shown. The viewer has no time label;
  the time of the row is in the render window.

The GUI chapter says both. What no test can do is look at a window: the columns and the size are for
the maintainer to see.

<a id="rg2-3-3-2"></a>
### RG2.3.3.2 — the most used settings on one representative model (2026-09-27, #2704)

**The model**: a ground with a checkerboard, a mass point with its spring-damper to the ground, a rigid
body on a revolute joint with a force and a torque, two sensors, and a two-node 2D ANCF cable - 21
items that draw, so that the fingerprint is per item. **The variants**: 28, each one setting (or a
setting and the one it needs switched on - the node tiling needs the nodes drawn as solids, the axes
tiling the joint axes), set on the same model and set back:

| group | settings |
|---|---|
| nodes | `show`, `showNumbers`, `drawNodesAsPoint`, `tiling`, `showBasis`, `defaultSize` |
| bodies | `show`, `showNumbers`, `beams.axialTiling` |
| connectors | `show`, `showNumbers`, `showJointAxes`, `general.axesTiling`, `springNumberOfWindings`, `defaultSize` |
| markers, loads, sensors | `show`, `showNumbers`, `drawSimplified`; `markers.defaultSize`, `loads.fixedLoadSize` |
| general | `circleTiling`, `cylinderTiling` |

**What it cannot see, measured**: the `view0.scene` settings - `showFaces`, `showFaceEdges`,
`showLines`, `facesTransparent`, `showMeshEdges`, `drawWorldBasis`, `drawCoordinateSystem` - change
nothing in `GetGraphicsData()`, because OpenGL applies them when it draws; `general.sphereTiling`
reaches no sphere of this model, and `bodies.beams.crossSectionTiling` no 2D beam. None of them is in
the case. The test also **fails if a variant stops changing the drawing**, so that a setting that
silently does nothing any more is seen.

**The size**: 28 full fingerprints would be about 400 KB. The reference stores the default and, per
variant, **only the groups and kinds of element that differ from it** (`VariantDelta`,
`ApplyDelta`, `CheckVariantsAgainstReference` in `graphicsRegression.py`), and every statistic is
rounded to 8 digits - what float32 data carries, and far below the tolerance of 1e-5: 50 KB for the
settings, 24 KB for the functions of RG2.3.3.1 (26 KB before the rounding). The whole test runs in
0.3 s.

**The fingerprint grew** by the sphere resolutions, the circle segments and the font sizes: the
tilings of circles and spheres change the resolution a sphere or circle is drawn with, not its count,
and the fingerprint did not hold it. The reference of RG2.3.3.1 was recorded again; its diff was the
additions and the rounding only.

<a id="rg13-4"></a>
### RG13.4 — the development documents per item type, drafted (2026-09-27, #2721)

The maintainer asked for a textual evaluation: what a reader needs to know about each kind of item,
seen from the tutorials and the Create functions, and what the generator writes around the
generated information. Six documents in `docs/revision/`: `itemDefinitionsDev.md` for what is common,
and one per kind - node, object, marker, load, sensor.

What was measured for them, beyond the table of RG13.1:

- **the Create functions**: a static scan of `mainSystemExtensions.py` - the 22 Create functions make
  36 different items; a static scan misses what an argument selects (the node of `CreateRigidBody`,
  the marker of `CreateForce`), so the documents propose recording it by running them;
- **the types**: the provided and requested types of all 97 items, from the definitions. They are
  the compatibility rules of the model and can be said in words on each page - except one rule that
  is only in C++: which node a node marker needs (`CSystem::CheckSystemIntegrity`);
- **the structure the equations texts already have**: the sub-headings per kind - bodies
  *Definition of quantities* and *Equations of motion*, connectors *Connector forces*, joints
  *Connector constraint equations* - so that the structure to prescribe is the one most object pages
  already follow; nodes and sensors have no sub-headings at all.

The largest gaps, by kind: the finite elements (`ObjectANCFBeam`, `ObjectBeamGeometricallyExact`
and `...2D`, `ObjectANCFThinPlate` have 4 to 15 words, `ObjectANCFCable` none); 9 of 16 nodes and 10
of 18 markers without text beyond the class description (`MarkerBodyRigid`, used in 127 scripts,
among them); sensors, whose class descriptions repeat the same three sentences eight times. Content
errors met on the way are listed in the documents and not fixed (e.g. `NodePointSlope1` gives a 3D
slope vector with two components).

<a id="rg12-27"></a>
### RG12.27 — store positions with Qt plot windows; the SolutionViewer when it is narrow (2026-09-27, #2722)

The maintainer tested 1.12.131 in Spyder with `tmp/experimenting/rigidBodyTutorial3.py`: renderer,
two `PlotSensor` windows, the SolutionViewer, then the settings dialog with *V* and **store
positions**. Two findings:

- **The button raised `TypeError: can only concatenate str (not "QRect") to str`** and opened
  nothing, so no plot window was ever stored - the `config.json` had the two dialogs and the render
  window, and no `PlotSensor`. Spyder's backend is QtAgg, and a Qt window has a `geometry()` as well,
  which returns a `QRect`: `PlotWindowGeometries()` and the storing of one plot window both asked
  *"has it a geometry()?"* first and took the Qt window for a tkinter one. Both now go through one
  function that tells tkinter by `wm_geometry` - as the placing of a window already did - and
  reads a Qt window by `x, y, width, height`. The automatic storing when a window closes had the same
  fault. The listing in the button writes each entry with `str()`, so that a window that reports
  something unexpected cannot stop the others. The test of the plot windows has a Qt stub now; the
  tkinter stub answers `wm_geometry`.
- **The SolutionViewer lost its third column** (*Static*, *Make mp4*) when it was narrower than its
  slider asks for - 1200 pixels at 500 steps and more. Tk takes the missing width from the weighted
  columns, down to nothing, and since RG12.26 every column right of the first was weighted.
  `ConfigureDialogColumns` (in `exudyn.interactive`) weights only the columns in which a slider
  starts, and gives every column the width of its buttons and labels as a minimum, so that only a
  slider gives. The sliders of the SolutionViewer and of the mode shapes span to the last column,
  which removes the empty cells right of them. A test lays out the viewer's grid in a withdrawn window
  700 pixels wide and finds all three button columns with a width - the same grid measured 0 for two
  of them before.

The window positions themselves are for the maintainer's hands again: the fix is tested on stubs.

<a id="rg12-28"></a>
### RG12.28 — PlotSensor opens at the stored size (2026-09-27, #2723)

The maintainer, after RG12.27: store positions stores the PlotSensor windows, position and size -
checked with different sizes in `config.json` - but a new PlotSensor opened at the stored position
and the **default** size. `__PlacePlotWindow` resized the window to the stored size, and two lines
later `PlotSensor` called `fig.set_size_inches(sizeInches, forward=True)` with the default 6.4 x 4.8
inches, and `forward=True` resizes the window. It had never shown, because no size was stored before
RG12.25.

`__PlacePlotWindow` returns now whether it applied a stored size, and `PlotSensor` sets the figure
size only if it did not, or if `sizeInches` was given in the script - the rule of the dialogs and
the render window: the script wins over what is stored. A figure with sub plots, which does not
place a window, is unchanged. A test places a stub Qt window with a stored size and a stub tkinter
window without one; on the screen it is for the maintainer to see.

<a id="rg13-5-0-1"></a>
### RG13.5.0.1 — overallDescription and detailedDescription (2026-09-27, #2724)

The maintainer: *"the classDescription and the equations fields in the items should be replaced into
overallDescription (brief description, summary) and detailedDescription. The reason for the split is
that the overall descr. is used for class, etc., while the full description goes into the docs. And
there is some auto-generated part before the details."*

The rename: 97 `overallDescription` and 69 `detailedDescription` in the five item definition files,
the keys the generators read (`itemDocsEmitter`, `itemHeaderEmitter`, `itemModel` - its mangling
rule and `OverallDescription()` - and `itemInterfaceEmitter`), the item report, and the field table
of `definitions/README.md`. **The regeneration is a no-op**, which is the proof that nothing else
moved. The structures keep `classDescription` (maintainer's decision): they have no detailed text
beside it.

With it, RG13.4 is complete for three kinds - nodes, loads and sensors are agreed - and RG13.5 is the
step that writes them. For loads the maintainer asked that the generalized forces keep their frames;
reading `CSystem::ComputeODE2SingleLoad` for that also showed that the static solver's load factor
multiplies every load **except** one with a user function (#603), which `loadDefinitionsDev.md` had
put the other way round.

<a id="rg2-3-3-3"></a>
### RG2.3.3.3 — graphics user functions, and the bug they showed (2026-09-27, #2704, #2726)

The case: a ground drawn by plain graphics data, a ground whose `graphicsDataUserFunction` moves a
brick with the time, a rigid body drawn by a user function under gravity, and a force whose
`loadVectorUserFunction` grows with the time, drawn with `loads.fixedLoadSize` off. The fingerprint
is taken at the start and after a dynamic solve of 0.5 s, and stored as the start and what changed.
The test asserts that the plain ground did not change and that the two user function objects and the
load did. The Python user functions are called by `GetGraphicsData()` itself - no renderer.

**On its first run the fingerprint had `system 1 None 0` and `system 2 None 0` in it** - groups for
a second and third system that the model does not have. `CallUserFunction` of `ObjectGround`,
`ObjectRigidBody`, `ObjectRigidBody2D` and `ObjectGenericODE2` passed the object number to
`EXUvis::AddBodyGraphicsData`, which takes an item ID - `Index2ItemID(itemNumber, ItemType::Object,
systemID)`, as every `UpdateGraphics` computes it - and decoded object 1 as system 1. The mouse
selection reads the same ID, so selecting a body drawn by a user function named a wrong item. Fixed
in `src/Graphics/VisualizationUserFunctions.cpp` (#2726); after it, the two objects are `Object 1`
and `Object 2`.

<a id="rg2-3-3-4"></a>
### RG2.3.3.4 — low-resolution raytracer images (2026-09-27, #2704)

Measured before deciding: the representative model of RG2.3.3.2 at 100 x 100 pixels takes **1 to 12
ms** per image through `SC.renderer.RedrawAndGetImage(useRaytracer=True)`, without a window, and two
runs give **identical** images. The settings the graphics data cannot see do change the image:
transparent faces 37 % of the pixels, face edges 11 %, no faces 31 %; `nodes.show` changes almost
nothing, because points are not raytraced. So there is **one set that always runs**, not two.

Two things had to go out of the image: the **texts** - the version number is one of them and would
change the reference with every commit (`raytracer.advanced.showText = False`) - and the **world
basis**, which decided the zoom (`drawWorldBasis`, `drawCoordinateSystem`). The four references -
default, `facesTransparent`, `showFaceEdges`, no faces - are PNG files of 0.2 to 1.1 KB, written and
read with `zlib` alone (`WritePNG`, `ReadPNG` in `graphicsRegression.py`), so a human can open them
and the test needs nothing beyond numpy. Two images agree if at most 1 % of the pixels differ by
more than 24 of 255 levels; that tolerance is a guess until the first run on linux. As in the other
cases, a variant that stops changing the image fails.

<a id="rg13-5-0-2"></a>
### RG13.5.0.2 — the page of a kind of item is a definition (2026-09-28, #2725)

The one text a kind of item had - a paragraph on the page `nodeIndex.md`, `sensorIndex.md`, ... -
was a Python dict inside the generator, `globalItemIntros` in `itemDocsEmitter.py`, where no check of
the descriptions reached it. It is `definitions/itemKindDefinitions.py` now, one
`ItemKindDefinition(kind, overallDescription, detailedDescription)` per page of a kind (the helper is
in `definitionTypes.py`), read by the emitter. The eleven paragraphs moved unchanged - the two
`\texttt{Marker}` as backtick spans and `\text{Marker}` as the word, which is what the converter made
of them - and **the regeneration is a no-op**. `checkDefinitions` reads the file with the others;
since a general section stands on the page of the kind under its title alone, its headings are
written with `##` there (`FILE_HEADING_LEVEL`), and `definitions/README.md` says so.

The `detailedDescription` is empty everywhere until RG13.5.1, .4 and .5 write the general sections;
where it is not, the page gets it after the paragraph and the list of items under a heading *Items*
of its own, so that the navigation does not hang the items under the last section of the general
text. The introduction of the whole chapter stays in the emitter: it is not a kind, and its
`\mybold` and `\refSection` would need the converter's help that a definition is not given.
