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

