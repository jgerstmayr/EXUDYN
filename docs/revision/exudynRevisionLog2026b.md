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
