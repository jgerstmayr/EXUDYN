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

