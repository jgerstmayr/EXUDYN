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
