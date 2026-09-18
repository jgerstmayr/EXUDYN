# Exudyn Revision Plan 2026

The steps of the v1.11.0 → v2.0 revision, by phase. General information, facts, decisions and the
numbering rules are in [`exudynRevisionInfo2026.md`](exudynRevisionInfo2026.md); what was done and
found, in the same order as here, is in [`exudynRevisionLog2026.md`](exudynRevisionLog2026.md).

Numbers are permanent: `R<phase>.<step>`, sub-steps `R4.10.3`, further splits `R4.10.3a`. A done
step keeps one line here (date, result, link to the log); an open step keeps its full text. Cite
steps outside these documents as "revision2026 step R4.10.3". New top-level steps and new phases are
proposed to the maintainer before they are written down; sub-steps may be added.

## R0 — Freeze current behaviour (~1 week) — do first  <!-- old Phase 0 -->

<a id="r0-1"></a>
**R0.1** **DONE 2026-09-09** — root `.gitignore` added. → [log](exudynRevisionLog2026.md#r0-1)

<a id="r0-2"></a>
**R0.2** **DONE 2026-09-11** — `tools/regenerate.py`, verified by fault injection and wired into CI as the `regenerated_files` job. → [log](exudynRevisionLog2026.md#r0-2)

<a id="r0-3"></a>
**R0.3** **DONE 2026-09-09** — the commit itself is the golden snapshot; a full regeneration produces no drift. → [log](exudynRevisionLog2026.md#r0-3)

<a id="r0-4"></a>
**R0.4** **DONE 2026-09-10** — baseline measured on every platform in active use. → [log](exudynRevisionLog2026.md#r0-4)

<a id="r0-5"></a>
**R0.5** **DONE 2026-09-09** — `.gitattributes` for line endings and binaries. → [log](exudynRevisionLog2026.md#r0-5)

<a id="r0-6"></a>
**R0.6** **DONE 2026-09-09** — explicit `encoding='utf8'` at all generator read and write sites. → [log](exudynRevisionLog2026.md#r0-6)

## R1 — Repository restart (~1 week, then a long freeze)  <!-- old Phase 2a -->

<a id="r1-1"></a>
**R1.1** **DONE 2026-09-09** — this working clone *is* that clone (depth-1 of GitHub `master` at `e44aca1`). → [log](exudynRevisionLog2026.md#r1-1)

<a id="r1-2"></a>
**R1.2** **DONE** — the old local repository is archived read-only (zipped, copy on the university server). → [log](exudynRevisionLog2026.md#r1-2)

<a id="r1-3"></a>
**R1.3** **DONE 2026-09-09** — `origin` is the internal GitLab, `github` the public repository. → [log](exudynRevisionLog2026.md#r1-3)

<a id="r1-4"></a>
**R1.4** **DONE 2026-09-10** — nothing experimental to move out of the tree. → [log](exudynRevisionLog2026.md#r1-4)

<a id="r1-5"></a>
**R1.5** **DONE 2026-09-09** — `tools/hooks/pre-push` refuses any ref but `master`, `release/*` and tags towards GitHub. → [log](exudynRevisionLog2026.md#r1-5)

<a id="r1-6"></a>
**R1.6** **DONE 2026-09-10** — one-time secret and PII scan of the tree to be published. → [log](exudynRevisionLog2026.md#r1-6)

<a id="r1-7"></a>
**R1.7** **PARTLY DONE 2026-09-09 (implementation), 2026-09-10 (first green pipeline).**
    `.gitlab-ci.yml` builds and tests manylinux wheels cp310-cp314 on the shared runners plus a
    `sphinx-build -W` docs job; all six jobs passed on the first run. Windows, macOS and aarch64
    are deliberately out of scope. → [log](exudynRevisionLog2026.md#r1-7)

    **Remaining: confirm a failing pipeline actually sends mail.** The weekly schedule is set
    (Settings → CI/CD  Schedules, target `v2-dev`, Sat 03:33) but lives only in the GitLab UI -
    if it is deleted, CI stops silently and the repository looks unchanged. An unnoticed red
    pipeline is the same as no pipeline, so this cannot be closed until a deliberate failure has
    been seen to arrive by mail.

<a id="r1-8"></a>
**R1.8** At v2.0: fast-forward `master`, push once with tags. Ordinary push; clones, permalinks and
    issue references stay valid; GitHub renders the layout change as renames.

<a id="r1-9"></a>
**R1.9** Retroactively tag past releases where the commits can be identified.

<a id="r1-10"></a>
**R1.10** *(phase R1, when there is material)* **Second internal GitLab repository for development-only
    Python.** Models that never make it to `Examples`, one-off study scripts, and internal
    experiments live there rather than in the public tree. Not created yet — do it when there is
    something to put in it, not before. Note the consequence for step R1.4: with a destination that
    is a *repository*, the sibling-directory and nested-repo shapes stop being the answer, and
    `experimental/` in `.gitignore` is only a safety net for work in progress.

## R2 — Make the wheel boring (~2 weeks)  <!-- old Phase 1 -->

Stay on setuptools. scikit-build-core or meson would cost the fast build and the zero-dependency
promise for no gain.

<a id="r2-1"></a>
**R2.1** **DONE 2026-09-11** — Static metadata moved to a `[project]` table in `main/pyproject.toml`. → [log](exudynRevisionLog2026.md#r2-1)

<a id="r2-2"></a>
**R2.2** **DONE 2026-09-11** — `pybind11<3.0` moved into `build-system.requires`. → [log](exudynRevisionLog2026.md#r2-2)

<a id="r2-3"></a>
**R2.3** **DONE 2026-09-11** — build parallelism: rescoped, setuptools parallelises across extensions only. → [log](exudynRevisionLog2026.md#r2-3)

<a id="r2-4"></a>
**R2.4** **DONE 2026-09-11** — `tools/gen_sources.py` derives `main/sources.json` from the `ClCompile` entries of `cppsrc.vcxproj`. → [log](exudynRevisionLog2026.md#r2-4)

<a id="r2-5"></a>
**R2.5** **DONE 2026-09-11** — the dead CMake build files deleted (405 lines). → [log](exudynRevisionLog2026.md#r2-5)

<a id="r2-6"></a>
**R2.6** **DONE 2026-09-12** — VS configurations reduced to `Debug|x64` and `Release|x64`. → [log](exudynRevisionLog2026.md#r2-6)

<a id="r2-7"></a>
**R2.7** **DONE 2026-09-12** — Ten of eleven `CIBW_*` variables moved into `[tool.cibuildwheel]` in `main/pyproject.toml`. → [log](exudynRevisionLog2026.md#r2-7)

<a id="r2-8"></a>
**R2.8** **DONE 2026-09-12** — the default `config` dict of `setup.py` is the schema. → [log](exudynRevisionLog2026.md#r2-8)

<a id="r2-9"></a>
**R2.9** **DONE 2026-09-12** — unaligned load/store in AVX loops over `LinkedDataVector`. → [log](exudynRevisionLog2026.md#r2-9)

<a id="r2-10"></a>
**R2.10** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-10) — **Two shipped variants,
    one meaning on every platform** (#2466): `exudynCPP` is baseline ISA everywhere, `exudynCPPfast`
    carries `__FAST_EXUDYN_LINALG` **and** AVX2, and `exudynCPPnoAVX` is gone together with the
    `sys.exudynCPUhasAVX2` switch. All 113 Windows reference values were re-measured on the
    baseline module: 85 moved, 33 of them past tolerance, which is the same set that
    `UnresolvedOnLinux()` lists — the Windows/Linux differences of #2379 WERE the AVX2 asymmetry.

<a id="r2-10-1"></a>
**R2.10.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-1) — *(sub-step of R2.10)*
    **`exudynCPPfast` no longer segfaults** (#2467). `MainObjectANCFThinPlate::SetWithDictionary`
    called `ParametersHaveChanged()` before the factory validated the item, and that computed the
    slope scaling from `GetCNodes()[-1]`. The regular module was saved by the array range check;
    the fast module, which compiles those out, died. The suite now completes under both modules.

<a id="r2-10-2"></a>
**R2.10.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-2) — *(sub-step of R2.10)*
    **`NGsolveCMStest` no longer rewrites its own committed input** (#2469). It saved the tracked
    `testData/netgenTestMesh.pkl` whenever the LOAD raised - which happens for reasons unrelated to
    the file being missing - and the result then moved by 2.4e-8. It now decides on
    `os.path.isfile`: written only when absent, and a file that cannot be loaded raises with the
    reason instead of being replaced. The tracked mesh is now an **`.npz`**, converted from the
    `.pkl` so the reference value is unchanged (maintainer, 2026-09-17).
<a id="r2-10-3"></a>
**R2.10.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-3) — *(sub-step of R2.10)*
    **A second reference set for the AVX2 module** (#2470), kept as an **update** to the baseline
    values: `AVX2ReferenceSolutionUpdate()` at the end of `runTestSuiteRefSol.py` holds only the 32
    values that actually move, so the other 100+ stay single-sourced. Both modules now pass the
    suite from one file. **The list is meant to shrink** (maintainer, 2026-09-17): each entry is to
    be removed either by finding the cause of the drift or by choosing model parameters that do not
    amplify roundoff — phase R10. It is ordered by drift, largest first, which is that work list.

<a id="r2-10-4"></a>
**R2.10.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-4) — *(sub-step of R2.10)*
    **`FEMinterface` files can be read by any module** (#2471): `postProcessingModes` stores the
    `outputVariableType` by name, so reading a file no longer imports `exudynCPP` to unpickle an
    enum. `NGsolveCMStest` still cannot leave `NotJudgedOutsideRegularModule()` - its tracked
    `testData/netgenTestMesh.pkl` was written in the old form and has to be regenerated first.

<a id="r2-10-5"></a>
**R2.10.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-5) — *(sub-step of R2.10;
    maintainer request 2026-09-17)* **The platform string names the architecture** (#2499):
    `Windows x86_64`, `MacOS arm64`, `MacOS x86_64`, `Linux arm64` instead of `Windows`, `MacOS`
    and `MacOS(ARM)`. It goes into the header of every solution, sensor, parameter variation and
    optimization file, so it is what a user sends with a bug report - and an Intel Mac could not be
    told from an Apple silicon one. The same step writes down why macOS builds no fast module.

<a id="r2-11"></a>
**R2.11** **DONE 2026-09-11** — Classifiers now 3.10–3.14, matching the wheels CI actually builds. → [log](exudynRevisionLog2026.md#r2-11)

<a id="r2-12"></a>
**R2.12** **DONE 2026-09-11** — installable extras `exudyn[tests]`, `exudyn[all]`, `exudyn[rl]`. → [log](exudynRevisionLog2026.md#r2-12)

<a id="r2-13"></a>
**R2.13** **DONE 2026-09-12** — the source distribution builds and installs. → [log](exudynRevisionLog2026.md#r2-13)

<a id="r2-14"></a>
**R2.14** **DONE 2026-09-12** — `setupPyConfig.json` retired. → [log](exudynRevisionLog2026.md#r2-14)

<a id="r2-15"></a>
**R2.15** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-15) — *(phase R2, small)* **Build and packaging hygiene** (#2372, #2380, #2387): SPDX licence, quiet Linux compile, project file entries checked.

<a id="r2-16"></a>
**R2.16** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-16) — *(phase R2, with R2.10)* **AVX2 on Linux decided by measurement** (#2396, #2397): stays OFF for the default wheel; if R2.10 builds a fast variant, it must use `-ffp-contract=off`.

<a id="r2-17"></a>
**R2.17** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r2-17) — *(phase R2 tooling, before the next header-only change)* **The wheel build does not see header
    changes** (#2427). setuptools recompiles a `.cpp` only when it is newer than its `.obj`, and it
    does not track included headers. In R4.4.3.2, `pip wheel . -w dist --no-deps` after a rewrite of
    `PybindUtilities.h` (included by 35 files) produced a `.pyd` with the same md5 as the build
    before, and the test suite passed against the old binary. Deleting
    `build/temp.win-amd64-cpython-313` forced the full compile (49 s). Until this is fixed, the build
    gate must remove that directory after a header change (seen again in step R4.13, #2448, duplicate). Fix: pass `depends=` (the headers) to
    the `Extension`, or let `setup.py` compare header times itself.

<a id="r2-17-1"></a>
**R2.17.1** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-17-1) — *(sub-step of R2.17)*
    **A compile-FLAG change now discards the previous build** (#2468). `setup.py` writes the
    effective options to `build/exudynBuildFlags.txt` and, when they differ, deletes the object
    files **and** the linked modules before compiling — #2427 covered headers, nothing covered
    flags, and `build/lib.*` kept the old `.pyd`.

## R3 — Repository shape (~1 week, one commit)  <!-- old Phase 2 -->

Do this **immediately after phase R1, before the months of v2.0 work**, and keep the moved files
byte-identical in that commit. Git stores no rename information; it infers renames by content
similarity at display time, so a file both moved and edited in one commit shows as delete+add.
Editing the vcxproj in the same commit is fine — it is modified, not moved.

<a id="r3-1"></a>
**R3.1** **DONE 2026-09-12** — Flattened: 1475 pure renames in one commit, repairs in the next. → [log](exudynRevisionLog2026.md#r3-1)

<a id="r3-2"></a>
**R3.2** **DONE 2026-09-12** — vcxproj paths (with step R3.1; nothing to fix). → [log](exudynRevisionLog2026.md#r3-2)

<a id="r3-3"></a>
**R3.3** **DONE 2026-09-12** — acceptance gate of the flattening, checked by hand in VS2022. → [log](exudynRevisionLog2026.md#r3-3)

<a id="r3-4"></a>
**R3.4** **DONE 2026-09-12** — one version source, written by `issueTracker.py`. → [log](exudynRevisionLog2026.md#r3-4)

<a id="r3-5"></a>
**R3.5** **DEFERRED 2026-09-12 — the maintainer's own work.** The original proposal (move `docs/demo`,
    22 MB, to release assets or git-lfs) is **withdrawn**: the `.gif` files are embedded in the
    GitHub front page, so moving them out of the repository would break it, and git-lfs would add
    a clone-time dependency for something the front page needs unconditionally. Instead the
    maintainer resized the images and animations in place and committed them on 2026-09-12
    (`57633e9`): 22 MB down to 11 MB, references in the docs and the landing page intact. Nothing
    for this plan to do; re-open only if the directory grows again.

<a id="r3-6"></a>
**R3.6** **DONE 2026-09-10** — `docs/howTo/` cut from 26 `.txt` files to 8 `.md`. → [log](exudynRevisionLog2026.md#r3-6)

<a id="r3-7"></a>
**R3.7** *(phase R3, then R2/R8)* **Revise `tools/buildAndGenerate/`.** **Cleanup DONE 2026-09-14 (with step R4.3
    part 2f):** all scripts rewritten portable (`condaActivate.bat`, paths from `%~dp0`), stale
    `main\` paths removed, `makeSphinxDoc.bat` added, `README.md` written, directory committed and
    the `.gitignore` entry removed. *Open:* absorption by step R8.2's `tools/release.py`.
    Original text: the 15 Windows/WSL batch scripts
    entered the tree in 2026-09 and were never on GitHub. Sequence deliberately: **first** decide
    the directory structure and move them with `git mv` in the phase R3 flattening commit (so the
    moves stay tracked), **then** check which scripts still work, keep only what is useful, strip
    what does not belong on GitHub — several contain hard-coded local paths such as
    `%USERPROFILE%\Anaconda\scripts\activate.bat`, and `execWithPythonVersion.bat` still ends
    with a `cd` into the long-gone `tools\makeWindowsBinaries\` — and add a README describing what
    each remaining script is for. Only then does step R8.2's `tools/release.py` absorb them; it must
    port these scripts, not reimplement alongside them.

    **Until this step runs, `tools/buildAndGenerate/` stays out of the repository.** It is listed in
    `.gitignore` with a pointer back here, so it neither clutters `git status` nor gets committed by
    accident with the hard-coded local paths still in it. **Committing it is part of this step** —
    remove the `.gitignore` entry at the same time as the cleanup, in the same commit.

    `tools/issueTracker/` is the opposite case and was committed on 2026-09-09 despite this step
    still being open, because it is *already in use* as the version source of truth (steps R8.3–R8.5
    will restructure it). Its `trackerlog.txt` was scanned for step R1.6 first: no credentials, no
    email addresses, no absolute local paths. `trackerlog.html` and `trackerlog_backup.txt` are
    regenerated on every tracker write and are ignored — `docs/RST/trackerlog.rst` and
    `docs/theDoc/trackerlog.tex` already carry the same content in tracked form.

    Related: `generateSetupFile.py` existed internally to generate `setup.py` by extracting the
    `.cpp` file names. It is deliberately **not** brought into this repository — step R2.4's
    `tools/gen_sources.py` supersedes it.

<a id="r3-8"></a>
**R3.8** *(phase R3, with the flattening)* **Give the logs their own directory.** Today they sit in three
    sibling directories next to the models — `TestSuiteLogs/`, `TestExamplesLogs/`,
    `PerformanceLogs/` — plus the `logsTmp/` added by step R5.10. Consolidate into a top-level `logs/`
    with one subdirectory per kind (`testmodels`, `examples`, `performance`) and a single shared
    `logs/tmp/`, which is what makes clearing scratch logs one delete. Do it inside the phase R3
    `git mv` commit so the moves stay tracked, and update the three `logFileName` expressions plus
    `testRunnerTools.tmpLogDir` in the same commit.

<a id="r3-9"></a>
**R3.9** *(phase R3, with the flattening)* **Move runners and helpers out of the model directories.**
    `TestModels/` currently mixes the models with `runTestSuite.py`, `runTestExamples.py`,
    `runPerformanceTests.py`, `runUnitTests.py`, `runTestSuiteRefSol.py`, `modelUnitTests.py` and
    `testRunnerTools.py`, which is why step R5.9's coverage check needs
    `NotTestModels()` to name seven files that are not tests. Separate them so the model
    directories contain models only. Performance
    models get their own directory as well; a model used for both performance and TestModels moves
    to performance. Sequence with step R5.9 — a completeness check over a directory of models only
    is far simpler than one that must know which files to ignore.

<a id="r3-10"></a>
**R3.10** **DONE 2026-09-10** — `docs/doxygen/` removed. → [log](exudynRevisionLog2026.md#r3-10)

## R4 — Code generation and docstrings (~6–8 weeks)  <!-- old Phase 3 -->

The core investment. Every step is validated byte-for-byte by step R0.2.

<a id="r4-1"></a>
**R4.1** **DONE 2026-09-14** — `objectDefinition.py` / `systemStructuresDefinition.py` converted into `definitions/`. → [log](exudynRevisionLog2026.md#r4-1)

<a id="r4-2"></a>
**R4.2** **DONE 2026-09-14** — validate the definitions on load (`definitionValidator.py`). → [log](exudynRevisionLog2026.md#r4-2)

<a id="r4-3"></a>
**R4.3** **DONE 2026-09-14** — re-point the generators at `definitions/`, then split them into emitters (`tools/generators/`). → [log](exudynRevisionLog2026.md#r4-3)

<a id="r4-4"></a>
**R4.4** **DONE 2026-09-15** — emitters read members directly (R4.4.1); one Python/C++ conversion layer `PyConversion.h` (R4.4.3); Jinja2 measured and not adopted (R4.4.2). → [log](exudynRevisionLog2026.md#r4-4)

<a id="r4-5"></a>
**R4.5** **DONE 2026-09-15** — MainSystem extensions bound by `@extends(exudyn.MainSystem)` and `install()` instead of copy-and-append (#2434). → [log](exudynRevisionLog2026.md#r4-5)

<a id="r4-6"></a>
**R4.6** **DONE 2026-09-15 (R4.6.1-R4.6.4, with 37).** Migrate the `#**` convention to **Google-style** docstrings across all 27 utility modules.
    Google style is mandatory project-wide (issue #2412); the rule itself has to be written into
    `docs/dev/CODING_STYLE.md`, `CONTRIBUTING.md`, `CLAUDE.md` and early in the user
    documentation, which is that issue's work and not this step's. The text inside the sections
    is Markdown (`$...$` math, no LaTeX macros; decision 2026-09-15, see step R7.1). Eight of the eleven tags map
    to standard sections — `function`/`class`/`classFunction` → summary, `input` → `Args`,
    `output` → `Returns`, `notes` → `Note`, `example` → `Example`. `belongsTo` disappears into
    `@extends`.

    **Reuse the converter that already exists.** `setup.py:476-527` loads
    `src/pythonGenerator/autoGenerateDocstrings.py` and runs `TreeConvert2Temp` over
    `python/exudyn/` at install time, converting the `#**` comments to docstrings on the way into
    the wheel. It already encodes the tag mapping, so it is the natural mechanical first pass for
    the migration: point it at the sources instead of a temporary tree, take the diff, hand-edit
    only where the output is not what a human would have written. Step R4.9 (delete it) then
    becomes the endpoint rather than a separate problem - once the sources hold real docstrings,
    the install-time transform has nothing left to do.

    **Sequence (maintainer decision 2026-09-15).** 36 cannot land alone: the docs emitters parse the
    `#**` comments, and `MainSystemExt.rst` (read by `pybindEmitter.py` and the `.pyi` stubs) comes
    from them. The `#**` sources at `158ccd9` are backed up outside the repository before any
    conversion (they also stay in git history). Sub-steps:
    - **R4.6.1 - malformed tags** (#2435). **DONE 2026-09-15.** Tags the parsers do not know were
      silently dropped from the documentation: `#**note` (9), `#**nodes` (3), `#**examples`,
      `#**compute`, `#**outputinput`, two `#**` continuation lines; five `#**function` without colon.
    - **R4.6.2/36c - docstring reader and converted sources** (#2436). **DONE 2026-09-15.** All 31
      modules (the 30 documented ones and `machines.py`) hold Google-style docstrings; `author`,
      `date` and `status` moved into `@docmeta(...)` (step R4.7, pulled in), and 10 functions with an
      untagged docstring are `@docmeta(public=False)`. `utilityDocsModel.py` reads docstrings and
      decorators with the standard library `ast`, not `griffe`: the reader needs no more than
      `ast` gives, and a dev dependency is not needed for it. Converted by a one-off lossless
      converter instead of `autoGenerateDocstrings.py`, which normalises whitespace. Details in
      the log.
    - **R4.6.4 - LaTeX to Markdown** inside the docstrings (#2437). **DONE 2026-09-15.** Step R4.6 is
      complete. → [log](exudynRevisionLog2026.md#r4-6-4)

<a id="r4-7"></a>
**R4.7** **DONE 2026-09-15 with R4.6.2/36c** — `@docmeta(author, date, status, public)` in `exudyn/docmeta.py`. → [log](exudynRevisionLog2026.md#r4-6)

<a id="r4-8"></a>
**R4.8** **DONE 2026-09-15** — `pydoclint` check of `python/exudyn` in GitLab CI, with a baseline (#2439). → [log](exudynRevisionLog2026.md#r4-8)

<a id="r4-9"></a>
**R4.9** **DONE 2026-09-15** — install-time docstring converter and `autoGenerateDocstrings.py` removed (#2439). → [log](exudynRevisionLog2026.md#r4-8)

<a id="r4-10"></a>
**R4.10** **DONE 2026-09-15 (R4.10.1-R4.10.4)** — *(phase R4, after R4.3)* **Expose the type information to Python** (issue #2411; was R4.1.6).
    Structures have a generated `GetDictionaryWithTypeInfo()`
    (`pythonAutoGenerateSystemStructures.py:531`) feeding the settings dialog (`GUI.py:323`);
    items have no equivalent, so the items dialog shows no types at all. A generated
    `python/exudyn/types/` subpackage carries, per parameter, the type, shape and range, plus
    `nodeType`, `requestedNodeType`, `requestedMarkerType` and `outputVariables` - none of
    which Python can see today. Generated from `definitions/`, so there is no second copy to
    drift. It is **not** put in `itemInterface.py`: that module is 390 KB and sits on the
    `exudyn.utilities` import path, so every user would pay for data wanted only by dialogs,
    checks and queries. `setup.py:530` already uses `find_namespace_packages`, so packaging
    needs no change.

    Second half of the same step: **`requestedNodeType` and `requestedMarkerType` become
    declared type lists** and the C++ accessor is generated from them, instead of being
    written as C++ inside the definition as it is today. The additive case is the common one.
    The known hard case must be designed for, not discovered later -
    `ObjectContactSphereSphere` (`src/Autogenerated/CObjectContactSphereSphere.h:184`) returns
    a base type plus **one conditional term governed by one parameter**:

    ```cpp
    return (Marker::Type)((Index)Marker::Position
                          + (parameters.dynamicFriction!=0)*(Index)Marker::Orientation);
    ```

    so the declaration needs a base list and an optional conditional list, e.g.
    `types=[MarkerPosition], conditional=[(MarkerOrientation, 'dynamicFriction != 0')]`.
    **Survey first** whether any case in the tree needs more than one condition; if none does,
    that is the whole grammar.
    - **R4.10.1 - declared requested types** (#2450). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-10-1).
      `ItemRequestedTypes(kind, types, conditional)`; one condition form in the tree.
    - **R4.10.2 - `Marker::Type` and `AccessFunctionType` generated** (#2451, was step R4.10.2; maintainer
      decision 2026-09-15: generated and in the Python interface, so that it becomes visible which
      objects, markers, connectors and loads combine). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-10-2).
      `LoadType`, `SensorType`, `CObjectType`, `JacobianType` stay hand-written.
    - **R4.10.3 - item type facts declared** (#2452). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-10-3).
      `ItemTypes` (34 nodes and markers), `ItemAccessFunctionTypes` (19 objects; the `.cpp` bodies
      removed). A marker needs no declaration: its Position/Orientation bits select the access function.
    - **R4.10.4 - `python/exudyn/types/`** (#2411). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-10-4).
      Generated `types/items.py` and tested queries (`MarkersForObject`, `ConnectorsForMarkers`, ...).

<a id="r4-11"></a>
**R4.11** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-11) — *(phase R4, before R4.3, small)* **Generator correctness** (#2414, #2415). The generator compares
    a C/Main/Visu header after cutting 7 lines, which removes `@class` and `@brief` as well as the
    two `@date` lines - so a changed class description is never written and `regenerate.py
    --check` cannot see it (#2415, `pythonAutoGenerateObjects.py:2029`); compare with
    `IsEqualIgnoringDateStrings` instead. Four items describe `AngularVelocityLocal` as a "3D
    velocity vector"; fix the text, check the neighbouring `AngularVelocity` texts (#2414). Both
    move published documentation, so they are gated as documentation changes.

<a id="r4-12"></a>
**R4.12** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-12); done as a comment, not as typedefs (`PReal` is the AVX packed-real macro) — *(phase R4, with R4.3)* **Keep the constrained types at the C++ boundary** (#2409). `PReal`,
    `UReal`, `PInt`, `UInt` are mapped to plain `Real` / `Index` in `typeConversion`, so the
    generated headers lose the intent that the Python-side `CheckForValid*` guards enforce. Add the
    four typedefs and emit the constrained name; no behaviour changes, the headers say more.

<a id="r4-13"></a>
**R4.13** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-13) — *(phase R4, after it)* **Remove `CFOptional`** (#2417). It wraps 448 parameter reads in
    `DictItemExists`, but nothing tests the behaviour, so it guarantees nothing. Parameters whose
    default value is not usable are the place where "required" belongs - as a checked property,
    not a hand-set flag. Update `modelUnitTests.py:184`, which relies on omitted parameters.

<a id="r4-14"></a>
**R4.14** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-14) — *(phase R4, with a large file move - R4.3 or later)* **Group `src/Autogenerated/` by item type.**
    The directory holds several hundred generated headers side by side. Item headers move into
    subdirectories `nodes/`, `objects/`, `markers/`, `loads/`, `sensors/`; the common generated
    files (`OutputVariableTypes.h`, `versionCpp.cpp`, the pybind and structure headers) stay in
    `src/Autogenerated/`. Include paths, `msvc/cppsrc.vcxproj`, `sources.json` and the tier lists
    in `tools/regenerate.py` follow. Do it when the generators are rewritten anyway, so the paths
    change once. Maintainer suggestion 2026-09-14; moving the generated files approved 2026-09-15.

> Migration note for 36: ~1,200 doc comments across 27 files. Convert mechanically with
> `autoGenerateDocstrings.py`, diff the generated RST against the pre-migration output, and
> hand-edit only where the diff is non-trivial.

<a id="r4-15"></a>
**R4.15** **DONE 2026-09-15** — `None` raises instead of converting. → [log](exudynRevisionLog2026.md#r4-15)

<a id="r4-16"></a>
**R4.16** **DONE 2026-09-15** — Item indices are rejected by `float` and `bool` parameters. → [log](exudynRevisionLog2026.md#r4-16)

<a id="r4-17"></a>
**R4.17** **DONE 2026-09-15** — item classes accept their own defaults; `CFMustBeGiven` for placeholder defaults. → [log](exudynRevisionLog2026.md#r4-17)

<a id="r4-18"></a>
**R4.18** **DONE 2026-09-14** — Generated files without a generator. → [log](exudynRevisionLog2026.md#r4-18)

<a id="r4-19"></a>
**R4.19** **DONE 2026-09-15** — One spelling per type: unify the item/structure exceptions of `typeModel.py`. → [log](exudynRevisionLog2026.md#r4-19)

<a id="r4-20"></a>
**R4.20** **DONE 2026-09-15** — `src/Autogenerated/StructuralElementsDataStructures.h` has no generator and no user. → [log](exudynRevisionLog2026.md#r4-20)

<a id="r4-21"></a>
**R4.21** **DONE 2026-09-15** — Wrong and stray comments in the generated item headers. → [log](exudynRevisionLog2026.md#r4-21)

<a id="r4-22"></a>
**R4.22** **DONE 2026-09-15 (R4.22.1-R4.22.3)** — *(phase R4, after R4.6)* **Star-import surface of the utility modules** (#2438, maintainer request
    2026-09-15). `from exudyn.utilities import *` exports everything `utilities.py` imports,
    including helpers such as `extends` (step R4.5) and `docmeta` (step R4.7), `np`, `sqrt` and
    `exudyn`, because no utility module has `__all__`; it also re-exports `basicUtilities`,
    `advancedUtilities`, `rigidBodyUtilities`, `graphicsDataUtilities` and `itemInterface` by star
    import, plus 23 deprecated `GraphicsData...` aliases. This is a v2.0 API change: user scripts
    relying on removed names break, so every removal is listed in the changelog (step R7.4).
    - **Ways to do it.** *Easy:* one `__all__` in `utilities.py` listing today's public names, helpers
      left out - nothing else changes. *Long-term ideal:* every utility module has its own
      `__all__`; `utilities.py` is a thin facade that only composes those lists; no module-level
      compatibility aliases; users are steered to the topical modules (`exudyn.graphics`,
      `exudyn.rigidBodyUtilities`, ...). *Recommended:* the ideal, reached in three sub-steps, each
      with a full suite run and the Examples/TestModels switched in the same commit:
    - **R4.22.1 - numpy-era vector helpers in `basicUtilities.py`** (#2442). **DONE 2026-09-15.**
      → [log](exudynRevisionLog2026.md#r4-22-1)
    - **R4.22.2 - `utilities.py`** (#2443). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-22-2).
      Maintainer decisions 2026-09-15: (1) Remove the 23 deprecated
      `GraphicsData...` aliases after replacing their uses by `exudyn.graphics.*`. (2) No new files;
      module names are final, since removing a function from a module later breaks user scripts:
      - the `@extends` functions (`CreateDistanceSensorGeometry`, `CreateDistanceSensor` with its
        helper, `DrawSystemGraph`) move to `mainSystemExtensions.py`;
      - the TCP/IP functions move to `advancedUtilities.py`;
      - **all other functions move to `basicUtilities.py`**, which may import numpy and exudyn from
        now on (so `exu.Print` stays), and becomes the module to import directly;
      - `advancedUtilities.py` keeps its functions (moving them would break explicit imports).
      `utilities.py` becomes the big import only: star imports and re-exports, no own functions,
      no `extends`. Import order without cycles: `exudyn/__init__` loads the C++ module first, then
      `mainSystemExtensions.py`, which imports `basicUtilities`/`advancedUtilities`/... but never
      `utilities.py`; `utilities.py` imports everything, including `mainSystemExtensions.py`.
      Signature defaults such as `exudyn.ConfigurationType.Current` are evaluated at import, which
      works because the C++ module is loaded before any utility module.
    - **R4.22.3 - `__all__`** (#2444). **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-22-3). Measure which implicitly exported names (`np`, `pi`, `sqrt`, `exudyn`,
      the itemInterface classes, ...) Examples and TestModels take from `from exudyn.utilities
      import *` (some do use `np` and `sqrt` that way; they get explicit imports). Then give every
      utility module an `__all__` and let `utilities.py` compose them. A checker keeps the lists
      complete: a small `ast` tool (like `tools/checkExtras.py`, run in CI) that fails if a public
      top-level function or class of a module - one with a docstring and not
      `@docmeta(public=False)`, the same rule the documentation reader uses - is missing from its
      `__all__`, or if `__all__` names something the module does not define.
    - **For every sub-step:** moved or removed names are checked in the package, TestModels,
      Examples and docs, and listed in the log table *API changes for the v2.0 release notes*,
      which step R7.4 carries into `CHANGELOG.md`.

<a id="r4-23"></a>
**R4.23** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-23) — *(phase R4, with the item emitters)* **Generated `itemInterface.py` docstrings do not match the
    signatures** (#2440). `pydoclint` reports 387 findings there: the `visualization` argument of
    every item class is undocumented, and the arguments carry types (`name (str): ...`) although
    the package convention has none. Fix in `itemInterfaceEmitter.py`, then drop the `exclude` in
    `[tool.pydoclint]`.

<a id="r4-24"></a>
**R4.24** **DONE 2026-09-15** — development environments from `[dependency-groups]` in `pyproject.toml`; `docs/requirements.txt` removed (#2441, maintainer request). → [log](exudynRevisionLog2026.md#r4-24)

<a id="r4-25"></a>
**R4.25** **DONE 2026-09-15** — structure members are in the Python interface by default; `SFPybind` on 845 of 883 members replaced by `SFNoPybind` on the other 38 (#2445, maintainer request). → [log](exudynRevisionLog2026.md#r4-25)

<a id="r4-26"></a>
**R4.26** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-26) — *(phase R4)* **Super element `Vshow` is False when left out of the dict** (#2447). `ObjectFFRF`,
    `ObjectFFRFreducedOrder`, `ObjectGenericODE2`, `ObjectKinematicTree`: `AddObject` without `Vshow`
    gives `False`, although `definitions/`, `itemInterface.py` and the generated Visu constructor say
    `True`. Found by the omit probe of `parameterConversionTest.py` (step R4.13), recorded in its reference.

## R5 — Testing (~3 weeks)  <!-- old Phase 4 -->

<a id="r5-1"></a>
**R5.1** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-1) — **`python/TestModels/test_testModels.py`**: one pytest case per model and mini example, sharing the reference values and tolerances with `runTestSuite.py`, which stays the gate runner.

<a id="r5-2"></a>
**R5.2** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-2) — **Fast vs slow as data**: `SlowTests()` and `OptionalPackageTests()` in `runTestSuiteRefSol.py` drive both `runTestSuite.py --fast` and the pytest markers; nightly stays the full set.

<a id="r5-3"></a>
**R5.3** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-3) — **The lest C++ unit tests
    can run again**: a `performUnitTests` build switch (off by default), `PERFORM_UNIT_TESTS` in the
    VS `Debug` configuration, and the two defects that would have skipped or crashed the suite's
    report (#2458).

<a id="r5-4"></a>
**R5.4** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-4) — **The AVX classes have
    unit tests**: `ResizableVectorParallel` and `LinkedDataVectorParallel`, every operation over 19
    lengths around the packet boundary and, for the linked one, every offset from 0 to `AVXRealSize`
    — the #2394 case. Verified by mutation. The remaining backfill is R5.4.1.

<a id="r5-4-1"></a>
**R5.4.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-1) — *(sub-step of R5.4)*
    **The matrix variants and the rigid-body/geometry group have unit tests** (#2472): 20 cases in
    `AllMatrixVariantsUnitTests.h` and `RigidBodyMathUnitTests.h`, property-based where a property
    exists, and validated by mutation — one of which the tests initially missed, which is how the
    weak case was found. Two defects found on the way: #2473 and #2474.

<a id="r5-4-2"></a>
**R5.4.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-2) — *(sub-step of R5.4)*
    **Symbolic has unit tests** (#2479): 11 cases in `SymbolicUnitTests.h` aimed at the expression
    TREE, which is what Python cannot reach - `Diff` by pointer identity, the value accessors, the
    non-recording path and the reference counting - plus an extension of `symbolicModuleTest.py`
    that keeps its reference value byte-identical. Defects found: #2480 and #2481.

<a id="r5-4-3"></a>
**R5.4.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-3) — *(sub-step of R5.4)*
    **`LinearSolver.h` has unit tests** (#2479): 10 cases in `LinearSolverUnitTests.h`; one system
    solved by all four variants (EXUdense, Eigen PartialPivLU, Eigen FullPivLU, EigenSparse), and
    the places where they deliberately differ pinned down. Defects found: #2482 and #2483.
<a id="r5-4-4"></a>
**R5.4.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-4) — *(sub-step of R5.4,
    from R5.4.1)* **`LinkedDataMatrix(const MatrixBase&)` did not compile** (#2473): it read the
    protected members of another object through a base-class reference. It now uses the public
    accessors, and the row-range constructor next to it has its first caller - the R5.4.1 test,
    which had to do the pointer arithmetic itself.

<a id="r5-4-5"></a>
**R5.4.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-5) — *(sub-step of R5.4,
    from R5.4.1)* **`MatrixContainer::MultMatrixVector` had two preconditions** (#2474): the dense
    path sized the result vector, the sparse path did not and then indexed into it. The sparse path
    now sizes it as well, and both products check their sizes.

<a id="r5-4-6"></a>
**R5.4.6** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-6) — *(sub-step of R5.4,
    from R5.4.5)* **`SparseTripletMatrix(rows, columns, triplets)` kept its size arguments**
    (#2476): it initialised both to 0 and never assigned them. Fixed and kept, per the maintainer,
    with the R5.4.1 tests as its first caller.

<a id="r5-4-7"></a>
**R5.4.7** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-7) — *(sub-step of R5.4,
    from R5.4.2)* **The symbolic headers include what they use** (#2480): `Symbolic.h`,
    `SymbolicVector.h` and `SymbolicMatrix.h` compiled only because `Symbolic.cpp` included
    pybind11 and `BasicLinalg.h` before them. Each is now self-sufficient, and the workaround in
    the R5.4.2 test header is gone - which is the proof.

<a id="r5-4-8"></a>
**R5.4.8** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-8) — *(sub-step of R5.4,
    from R5.4.2)* **A failed symbolic operation frees its nodes** (#2481): `SReal(ExpressionBase*)`
    evaluates eagerly to cache the value, and an error inside `Evaluate()` used to escape before
    any object owned the allocation. It now releases the tree exactly as the destructor would.
<a id="r5-4-9"></a>
**R5.4.9** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-9) — *(sub-step of R5.4,
    from R5.4.3)* **The sparse factorization stopped inventing a causing row** (#2482): it returned
    `solver.info() - 1`, an Eigen status code, so the solver printed "causing system equation
    number = 0" for every singular sparse system. It now returns `NumberOfRows()`, the documented
    "row unknown" answer, and the solver prints no row.

<a id="r5-4-10"></a>
**R5.4.10** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-10) — *(sub-step of R5.4,
    from R5.4.3)* **`LinearSolverType.EigenDense` says what it does not detect** (#2483). The
    behaviour was never a defect - FullPivLU is the `ignoreSingularJacobian=True` least-squares
    path and PartialPivLU has no invertibility check in Eigen - so the step became one sentence in
    the enum description.
<a id="r5-4-11"></a>
**R5.4.11** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-11) — *(sub-step of R5.4;
    maintainer request 2026-09-17)* **`pythonTests.cpp` removed** (#2484): 849 lines of manual
    development tests, of which `PyTest()` was commented out entirely and `CreateTestSystem` -
    written before `exudyn.demos` existed - built a model from a `py::exec` string in the
    pre-`exudyn` API. Both were bound only outside a release build. The file, its header
    `PybindTests.h`, the two bindings and the project entries are gone.
<a id="r5-4-12"></a>
**R5.4.12** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-12) — *(sub-step of R5.4;
    maintainer request 2026-09-17)* **The C++ usage demo of the symbolic types became a readable
    header** (#2485): `PyTest_unused()` - six `if(false)` blocks in one dead function at the end of
    `Symbolic.cpp` - is now `src/Linalg/symbolicCppDemo.h`, one named function per topic, included
    by `Symbolic.cpp` so that it keeps compiling, called by nothing.
<a id="r5-5"></a>
**R5.5** **DONE 2026-09-17** — *(phase R5, tooling; decisions D1-D5 answered by the maintainer on
    2026-09-17)* **A linter and a type check for the Python side.** Half A is R5.5.3 (ruff), half B
    is R5.5.4 (stubtest and the PEP 561 marker); R5.5.1, R5.5.2, R5.5.5 and R5.5.6 are the defects
    the two halves uncovered. The text below is kept because it records what the decisions meant;
    it was written earlier the same day, when neither half had been started, because the decisions
    had been stated in a shorthand that assumed knowledge of the tools.

    **Half A - ruff over the shipped package `python/exudyn/`**

    *What ruff is.* One program that reads Python source and reports suspicious lines. It replaces
    what used to be three separate tools - pyflakes (real mistakes), pycodestyle (layout) and isort
    (import order) - by re-implementing their rules in Rust, fast enough to check all 34 000 lines
    in well under a second. It **changes nothing**: `ruff check` only reports. (`ruff format` is a
    separate, opt-in code formatter and is explicitly **not** part of this step - it would rewrite
    every file in the package.) It is configured by a few lines in `pyproject.toml`, and a single
    finding on a single line is silenced by a trailing `# noqa: F401`.

    *What "the default E/F rule set" means.* Every rule has a letter prefix naming the tool it comes
    from, plus a number:

    | prefix | what it is | examples |
    |---|---|---|
    | **F** | pyflakes - *real defects* | **F821** an undefined name (a typo that only raises when that line is finally reached); **F811** a name defined twice, the second silently winning; **F401** imported and never used |
    | **E4, E7, E9** | the part of pycodestyle that is not about whitespace | **E402** import not at the top of the file; **E711** `x == None` instead of `x is None`; **E722** a bare `except:`, which also swallows Ctrl+C and MemoryError; **E999** the file does not parse at all |
    | E1, E2, E3 | pycodestyle layout: indentation, blank lines, spaces around operators | **off by default and they must stay off** - thousands of findings that say nothing about correctness |
    | B, UP, I, N, ... | optional families: bugbear, pyupgrade, isort, naming, ... | opt-in; none active by default |

    So **"the default" = F + E4 + E7 + E9**: what is broken or misleading, nothing about formatting.
    That is what ruff checks when a project configures no rules at all.

    *What it would find here.* ruff is not installed in `venvExuP313`, so this was estimated on
    2026-09-17 with a stdlib-AST script over the 40 files of `python/exudyn/` (ruff's own count will
    differ, mainly because it also finds F811/F821/E402, which the script does not look for):

    | rule | count | character |
    |---|---|---|
    | E722 bare `except:` | 60 | **not mechanical** - each one needs a decision on which exception was meant |
    | E711 / E712 `== None`, `== True` | 63 | mechanical and safe |
    | F403 `from module import *` | 26 | mostly deliberate re-export inside the package |
    | F401 unused import | 14 | mechanical |
    | E731, F541 | 2 | cosmetic |
    | **total** | **~165** | over 34 000 lines - the package is in good shape |

    *The three decisions, restated as questions with what each answer costs* - **all five (D1-D5)
    were answered by the maintainer on 2026-09-17 with the recommendation given in each case**:

    - **D1 - which rules?** (a) the default F + E4/E7/E9, ~165 findings, all of them about
      correctness; (b) the default plus a small opt-in family such as **B** (flake8-bugbear: mutable
      default arguments, `except` order, loop-variable capture - real bug patterns, maybe 20-50 more
      findings); (c) more than that. *Recommendation: (a) now, (b) as a later sub-step once the
      default set is clean and stays clean.*
    - **D2 - fix or freeze?** A *baseline* is the pattern already used for `pydoclint`: the current
      findings are written to `tools/ci/pydoclintBaseline.txt` and the check fails only on findings
      that are **not** in that file, so old debt is tolerated and new debt is blocked. The
      alternative is to fix everything once and have no baseline file at all. *Recommendation:
      split by character - fix the ~79 mechanical ones (E711/E712/F401) outright in one reviewable
      commit, baseline the 60 bare `except:` and the 26 star-imports, and work the baseline down in
      later steps.* A baseline that is never reduced is just a list of things nobody will fix.
    - **D3 - where does it run?** (a) in the commit gate next to `checkAll.py --check`, so nothing
      is committed that fails it; (b) in CI only; (c) both. The check takes well under a second, so
      cost is not the argument - the argument is that a gate stops work, and a CI job does not.
      *Recommendation: (a) - `tools/checkPython.py --check`, alongside the existing checkers, and
      the same script in CI.*

    **Half B - a type check whose purpose is the stubs**

    *How the stubs are made today.* `python/exudyn/__init__.pyi` (249 KB) and
    `python/exudyn/symbolic.pyi` (13 KB) are the files an IDE reads to know what the C++ module
    offers. Five fragments feed them: the hand-written `tools/generators/stubHeader.pyi`, and four
    generated from `definitions/` by `pybindEmitter.py` (`stubAutoBindings.pyi`, `stubEnums.pyi`,
    `stubSymbolic.pyi`), `structureStubEmitter.py` (`stubSystemStructures.pyi`) and
    `mainSystemExtensionDocsEmitter.py` (`stubAutoBindingsExt.pyi`).
    `tools/generators/createStubFiles.py` (97 lines) then merges them **by string concatenation**:
    a line-based state machine that starts collecting when a line begins with `class `, and decides
    the class has ended as soon as it sees a non-empty line that does not start with four spaces.
    Nothing parses the result.

    *Measured 2026-09-17 - the merged stub is not valid Python.* All five fragments parse; the
    merged `python/exudyn/__init__.pyi` does **not**:

    ```
    File "python/exudyn/__init__.pyi", line 67
        """measure 3D position, e.g., of node or body"""
                   ^ SyntaxError: invalid decimal literal
    ```

    The cause is one line of documentation text. The class docstring of `OutputVariableType` has a
    continuation line starting at column 0 ("Available output variables and the interpreation ..."),
    which inside a triple-quoted string is harmless - but the merger's rule sees a column-0 line and
    concludes the class ended. The remaining class body is written to the top level, the class block
    is closed mid-docstring, and the halves land in the merged file out of order. Indenting that
    single line by four spaces makes the whole 249 KB file parse (verified by re-running the merge
    in memory). `symbolic.pyi`, which is a plain two-file concatenation, is fine.

    *Measured the same day - what the stubs do not describe.* Comparing the merged stub against the
    imported module: classes are covered well (76 of them), **module-level functions are largely
    absent** - `StartRenderer`, `StopRenderer`, `InfoStat`, `GetVersionString`,
    `SetOutputPrecision`, `SetWriteToConsole`, `SuppressWarnings` and others appear nowhere in the
    stub - the settings classes are missing `GetDictionary`/`SetDictionary` throughout, and
    `exudyn.symbolic` is missing `atan2`, `variables`, `Matrix.Get` and `UserFunction.Evaluate`.

    *The right tool is named.* `mypy` ships **`stubtest`** (`python -m mypy.stubtest exudyn`), whose
    single purpose is to import a module and compare it against its stub, reporting names present in
    one and not the other and signatures that disagree. That is exactly the job described here.
    `pyright` is a different job: it type-checks *source*, which would mean type-checking the whole
    utility package - valuable, but a much larger and noisier undertaking.

    *The two decisions:*

    - **D4 - which checker, and how much?** (a) `stubtest` only, i.e. stub-vs-module agreement, the
      question the stubs exist to answer; (b) `pyright`/`mypy` over `python/exudyn/` as source as
      well. *Recommendation: (a). (b) is a separate later step, because the utility package is
      untyped and a source type check on untyped code mostly reports the absence of annotations.*
    - **D5 - baseline again?** stubtest will report a few hundred names on the first run. Same
      answer as D2: freeze the first run as a baseline, fix the classes of finding that are
      systematic (the missing module-level functions, the missing `GetDictionary`/`SetDictionary`)
      as their own sub-steps, and let the baseline shrink.

    Sub-steps R5.5.1 and R5.5.2 below are defects found while writing this and are independent of
    every decision above. Half A is carried out in R5.5.3, half B in R5.5.4.

<a id="r5-5-1"></a>
**R5.5.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-1) — *(sub-step of R5.5)*
    **The generated `__init__.pyi` parses again, and the generator now checks** (#2486). The cause
    was not the documentation text but the emitter: a docstring *summary* was written without
    indentation, and a summary ends at the first ". " - which may come after a line break.
    `createStubFiles.py` now `ast.parse()`s both stub files and refuses to write an invalid one.

<a id="r5-5-2"></a>
**R5.5.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-2) — *(sub-step of R5.5)*
    **The stubs describe the module-level functions and the settings dictionaries** (#2490). The
    cause was `addDocu=False`, which suppressed the `.pyi` entry together with the documentation,
    and a `GetDictionary`/`SetDictionary` pair that the C++ emitter adds but the stub emitter did
    not mirror. 14 functions and 43 class pairs.

<a id="r5-5-3"></a>
**R5.5.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-3) — *(sub-step of R5.5,
    half A; decisions D1-D3)* **ruff runs over `python/exudyn/`** (#2487): rule set written down in
    `pyproject.toml`, `tools/checkPython.py` judging it against `tools/ci/ruffBaseline.txt`, in the
    commit gate. 335 findings at introduction, **107 fixed outright**, 228 tolerated and meant to
    shrink. The measured count was 565, not the ~165 estimated when R5.5 was written.

<a id="r5-5-4"></a>
**R5.5.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-4) — *(sub-step of R5.5,
    half B; decisions D4-D5, and the `py.typed` question answered with option (a))* **`stubtest`
    compares the stubs against the module**: `tools/checkPython.py --stubs`, two allowlists
    (curated noise, generated backlog of 271), and the PEP 561 marker is shipped - **after** R5.5.2
    closed the gaps that would have turned correct user code into reported errors.

<a id="r5-5-5"></a>
**R5.5.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-5) — *(sub-step of R5.5;
    found by the linter of R5.5.3)* **Four undefined names that raise `NameError` when their code
    path is reached** (#2488): three `exudyn.Print` in `lieGroupIntegration.py`, which imports the
    module as `exu`, and `SC.renderer.Start()` in `roboticsCore.py`, where the member is `self.SC`.
    Running the first path then showed a second defect behind it.

<a id="r5-5-6"></a>
**R5.5.6** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-6) — *(sub-step of R5.5)*
    **`robotics/future.py` imports `graphics`** (#2489), and its `MakeCorkeRobot` raises instead of
    returning an undefined name - the same pattern as R5.5.5, one function further.

<a id="r5-6"></a>
**R5.6** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-6) — *(phase R5)* **An ASan/UBSan
    Linux job.** For a C++ library invoking arbitrary user callbacks this catches the class of bug
    users report as "it crashed with no message".

    `tools/ci/buildSanitizers.sh` builds with `-fsanitize=address,undefined -O1 -g` and runs the
    full test suite against the result; `sanitizers_linux` in `.gitlab-ci.yml` runs it weekly. The
    job is **GitLab**, not GitHub: GitHub Actions only fire on pushes to master and on pull
    requests, so during the freeze they never run (step R1.7).

    **No `setup.py` change was needed**: the flags travel through `EXUDYN_EXTRA_COMPILE_ARGS` and
    `EXUDYN_EXTRA_LINK_ARGS`, which `setup.py` already appends to every extension.

    **The step paid for itself before a single sanitizer check ran** (#2506, fixed here): at `-O1`
    the module would not even load, because `RaytracingSettings::maxNThreads` is declared
    `static const` with an in-class initializer and never defined, while `Clamp()` binds a
    reference to it. `-O3` folds the constant and hides it; every release build has been linking on
    that accident.

    **Measured 2026-09-18**, the whole suite under both sanitizers, in WSL: **0 AddressSanitizer
    errors and 0 UndefinedBehaviorSanitizer reports** over 114 test models and 23 mini examples.
    The suite's own failures in that run are **not** memory findings - the build is `-O1` on Linux
    in an environment without scipy/NGsolve, so reference values differ and ~10 models cannot run -
    and the script therefore reports the suite exit code without failing on it. Correctness is
    `wheels_linux`' job; memory safety is this one's.

<a id="r5-6-1"></a>
**R5.6.1** *(sub-step of R5.6; first CI run seen 2026-09-18)* **Let the sanitizer job go red.**
    It is `allow_failure: true` for now. The **first GitLab run failed for a packaging reason, not
    a sanitizer one**: the job installed `setuptools` and `wheel` but not `pybind11`, and
    `buildSanitizers.sh` builds with `--no-build-isolation`, so pip does not fetch
    `[build-system] requires` itself. Everything up to that point worked - `apt-get` brought in
    gcc 14.2 and the matching libasan, and the script found both. Fixed by installing
    `pybind11<3.0` in the job and by checking the three build modules up front, so the next such
    failure is one line instead of line 443 of a pip traceback.

    Still open: after a run that actually reaches the suite, either flip `allow_failure` to `false`
    or baseline what is found. A job that may be red forever teaches people to ignore it.

<a id="r5-7"></a>
**R5.7** **DONE** — rename `pytest.py` - done differently in step R3.1 (`python/pytestTemplate.py`). → [log](exudynRevisionLog2026.md#r5-7)

<a id="r5-8"></a>
**R5.8** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-8) — *(phase R5)* **`runTestSuite.py --parallel[=N]`**: every model in its own interpreter, 22 s → 9-11 s; serial stays the default for the commit gate.

<a id="r5-9"></a>
**R5.9** **DONE 2026-09-11** — Complete and verify the test list. → [log](exudynRevisionLog2026.md#r5-9)

<a id="r5-9-1"></a>
**R5.9.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-9-1) — *(sub-step of R5.9)*
    **`symbolicModuleTest` fails with numpy 2.2**
    (#2501). The vector/matrix section compares the symbolic result against the numpy result with an
    **absolute** tolerance, `np.linalg.norm(res[0]-res[1]) > 1e-15`, on a value of magnitude
    `9.7476` - where one ulp is `1.8e-15`. The tolerance is below the representable resolution, so
    whether the test passes depends on the last bit of a numpy sum. Measured with the **same exudyn
    binary** (md5 identical in both environments): numpy 2.4.6 passes, numpy 2.2.4 differs by
    `1.78e-15`, counted once per recording mode. `cntWrong` is added to the test result since #2479,
    so the model returns `2.948...` against a reference of `0.948...` and the suite fails with error
    2.0. The comparison needs a relative tolerance.

<a id="r5-9-2"></a>
**R5.9.2** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-9-2) — *(sub-step of R5.9)*
    **A reference value depended on the numpy version** (#2502). Same binary (md5 identical),
    same source, same machine, same Python: `sliderCrank3Dbenchmark.py` returned
    `7.256859912845965` under numpy 2.4.6 and `7.256859914829453` under numpy 2.2.4, relative
    `2.7e-10` against a tolerance of `5e-14`.

    **Root cause.** Not the model and not the solver: the geometry setup and the whole assembled
    system were bit-identical under both, and only two marker `localPosition` values differed. They
    come from the 3x3 product every `Create*Joint` uses to convert the joint position into body
    coordinates. numpy does not fix the summation order of a small matrix product and changed it
    between releases - for a component that is analytically zero, 2.2.4 returns `0.0` and 2.4.6
    returns `-2.9e-19`. Proved by running the model on ONE interpreter with the other numpy on
    `PYTHONPATH`: the result flips. Recorded as fact 28.

    **Fixed by option 1** (maintainer, 2026-09-18): the 22 products in the `Create*Joint` helpers go
    through two private, written-out helpers, `_MatVec3` and `_MatMul3x3`, whose summation order is
    fixed by the source instead of by whichever kernel numpy picks. Cost: a Python loop over nine
    terms, once per joint at model-build time.

    **Result**: the two marker positions are now **bit-identical** under both numpy versions, and so
    is the model - `7.256859914829453` in `venvP313` (numpy 2.2.4) and `venvExuP313` (numpy 2.4.6)
    alike. Exactly **one** reference value moved, as the experiment had predicted, plus its AVX2
    counterpart, which the same Python-side change moves identically. Both suites now pass in both
    environments and with both modules.

    **Deliberately not changed**: the same pattern appears about eighty more times in `FEM.py`,
    `kinematicTree.py` and elsewhere. Nothing measured makes them matter, and a public utility would
    be new API - so the helpers stay private and local, with the reason written where they are.

<a id="r5-10"></a>
**R5.10** **DONE 2026-09-10** — `testRunnerTools.ResolveLogFile()` decides the log target before the first write. → [log](exudynRevisionLog2026.md#r5-10)

<a id="r5-11"></a>
**R5.11** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-11) — *(phase R5, release
    testing)* **Every compiled variant is covered by the release tests** (#2495): `EXUDYN_MODULE=fast`
    and `--fast-module` run the suite and the performance tests against `exudynCPPfast`, which had
    never been tested. The `-noavx` half of the original step is **dropped**: step R2.10 removed
    that module, so there are two variants, not three.

<a id="r5-11-1"></a>
**R5.11.1** **DONE 2026-09-17** — *(sub-step of R5.11; maintainer decision 2026-09-16)* **How much
    of the matrix the fast variant needs.** The decision stands as made and is now written where it
    is used, in `docs/dev/WORKFLOW.md` under "Release testing matrix", with `-noavx` removed: full
    suite on the default module for every supported Python version, full suite `--fast-module` on
    the oldest and the second newest (today 3.10 and 3.13), examples on one version, and
    `runPerformanceTests.py` both ways. The newest version is deliberately not the fast-mode
    target: right after a release its packages are the unstable part.

<a id="r5-11-2"></a>
**R5.11.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-11-2) — *(sub-step of R5.11;
    found by a maintainer question about the version string)* **"Is this the fast module" is not
    "does it have AVX2"** (#2496): the guard and the log marker of R5.11 asked the second question,
    so on macOS and in any `--no-avx2` build - where `exudynCPPfast` is built without vector
    extensions - `--fast-module` would have aborted, and the log would have carried no marker.

<a id="r5-12"></a>
**R5.12** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-12) — *(phase R5, small)*
    **Test and example hygiene** (#2368, #2377). `ANCFbeltDrive` was retuned by the maintainer and
    enters the suite - single-threaded, because with 4 threads it was not reproducible to the suite
    tolerance. Two of the three phantom imports are repaired and gone from
    `knownMissingLocalModules`; `RL_Spot` remains and needs a decision.

<a id="r5-12-1"></a>
**R5.12.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-12-1) — *(sub-step of
    R5.12)* **`CompositionRuleForRotationVectors` returns 2π instead of 0** (#2494):
    composing π·n with itself gives a vector of norm 2π rather than 0. Both describe the identity
    rotation; the 2π one is simply not the principal representative.

    **The C++ was checked against, as the maintainer asked.**
    `EXUlie::CompositionRotationVector` (`src/Linalg/RigidBodyMath.h:1171`) is the *same formula*,
    term for term, and returns 2π as well: `w = pi - 2*atan2(x, xTemp)` is `2*acos(x)`, which for
    `x = cos(w/2) = -1` is 2π. So this was never a Python port that drifted - it is a property both
    share.

    **Decision (maintainer, 2026-09-18): accept it and document it.** The formulas take noise and
    pass it on rather than snapping to a boundary; that keeps every existing result unchanged, and
    a caller who needs the principal range maps it himself - for `w > π`, use `2π - w` about the
    negated axis. Written into the Python docstring and the C++ comment, each naming the other, so
    neither can be "fixed" later in ignorance of the other.

    **One real difference was found on the way and fixed**: the C++ computes
    `sqrt(fabs(1 - x*x))` with the comment *"fabs added, because term may be slightly smaller than
    zero"*, and the Python had no guard - so the shipped Python **raised
    `ValueError: math domain error`** for exactly the case of #2494, where the C++ returned 2π·n.
    Ported, with the C++ named in the comment.

    **`LieGroupIntegrationUnitTests.py` now passes 10 of 10.** TEST 2 compared against the Matlab
    principal-range answer `[0,0,0]`; it now checks what is actually being claimed - that the
    composed vector describes the **identity rotation**, whatever representative it uses - which is
    the statement with meaning and survives a later change of convention. Measured: for
    `n = [1,1,1]/sqrt(3)` the norm is 2π to `8.9e-16` and `ExpSO3` is the identity to `3.7e-16`.
    Accuracy at that singularity is axis-dependent (about `4e-8` for `n = [0,0,1]`), which the
    docstring says. The file stays in `DeliberatelyNotRun()` for the one remaining reason: it
    PRINTS its results instead of setting `testResult`.

<a id="r5-13"></a>
**R5.13** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-13) — *(phase R5, with R5.8 and R5.9)* **Test-suite output goes to its own directory** (#2418, #2454): `exudyn.config.outputDirectory` and one output directory per model; no model writes next to itself any more.

<a id="r5-13-1"></a>
**R5.13.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-1) — *(sub-step of R5.13)*
    **Stop writing what nothing reads** (#2492): fifteen models wrote the coordinates solution file
    on every run although only the SolutionViewer reads it, and three sensors still wrote to files.
    74 → 64 solution files and 11 MB → 8.0 MB per suite run. The sensor half of the original step
    text turned out to be done already, by R5.13 and R5.13.2; the examples half became R5.13.3.

<a id="r5-13-2"></a>
**R5.13.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-2) — *(sub-step of R5.13)*
    **Five examples wrote sensor output next to themselves** (#2475): `beltDriveALE` and
    `beltDriveReevingSystem` into `solutionDelete/`, the latter also `solution_nosync/`,
    `rigidBodyIMUtest` into `solutionIMU<mode>/`, and the two `sliderCrank3DwithANCFbeltDrive`
    examples into the current directory plus `plots/`. Only `solution/` is ignored, so each direct
    run left untracked, unignored files behind. All of it now goes through `solution/`, and the
    reads through `OutputFilePath`.

<a id="r5-13-3"></a>
**R5.13.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-3) — *(sub-step of R5.13,
    split out of R5.13.1)* **Generated FEM data leaves the tracked input directory** (#2491): twelve
    examples wrote meshes and FEM data into `testData/`, so every examples run left untracked
    `.npz`/`.hdf5` files next to the tracked inputs. They now go through `OutputFilePath` into
    `solution/`.

<a id="r5-13-4"></a>
**R5.13.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-4) — *(sub-step of R5.13;
    maintainer request 2026-09-17)* **Every writer creates its own output directory** (#2493):
    `SaveDictToHDF5` raised `FileNotFoundError` instead, while four other writers each carried
    their own copy of the same `try/except os.makedirs` block. One function,
    `basicUtilities.CreateDirectoryForFile`. The two tracked `.npy` reference meshes, which nothing
    could load any more, are deleted (maintainer approval 2026-09-17).

<a id="r5-13-5"></a>
**R5.13.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-5) — *(sub-step of R5.13;
    found while answering a maintainer question)* **The suite log is one file again when
    `EXUDYN_OUTPUTDIRECTORY` is set** (#2500): the body went to the output directory and the summary
    to `python/TestSuiteLogs`, because the suite cleared `outputDirectory` before re-opening the log.
    In the same step the three runner `.bat` files pass their extra arguments on, so `--fast-module`
    and `--parallel` can be reached from them at all.

<a id="r5-14"></a>
**R5.14** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-14) — **Dev tools are declared**:
    a `test` dependency group (`pytest`, `pytest-xdist`), the `build` group matched to
    `build-system.requires` and to the cibuildwheel version CI pins.

<a id="r5-14-1"></a>
**R5.14.1** *(sub-step of R5.14; NEEDS APPROVAL - touches `.github/workflows/`)* **One set of action
    versions.** `wheels.yml` uses `actions/setup-python@v6`, `documentation.yaml` still
    `actions/checkout@v3` and `actions/setup-python@v4` (#2463). Raise both to the same version and
    note in the file why they are pinned at all.

<a id="r5-15"></a>
**R5.15** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-15) — **Performance suite reports
    single runs**: every run appends solver time, result and a run name to `exudynTestGlobals.timings`,
    reported and judged one by one; `perfLargeMassSpringChain` is a rigid body chain over 1000/5000/20000
    bodies, explicit and implicit; `generalContactSpheresTest` runs with 1, 4 and 8 threads.

<a id="r5-16"></a>
**R5.16** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-16) — **Examples run in parallel**
    with a short timeout: each example in its own interpreter and its own output directory, a timeout
    after the solver was reached counts as a pass. 360 s → 49 s.

<a id="r5-17"></a>
**R5.17** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-17) — *(phase R5, after R5.16)* **A switch that stops Exudyn opening windows**, so that a model
    or an example run outside the test suite does not pop up the renderer - and so that the runners
    can stop rewriting the source to prevent it.

    **Where it lives**: a new group `exu.special.userInterface`, next to the existing
    `special.solver` and `special.exceptions` (`PySpecialSolver`, `PySpecialExceptions` in
    `src/Main/Experimental.h`). Not for regular users - that is what `special` means.

    **The flags**, all default `False`:

    | flag | effect |
    |---|---|
    | `suppressRenderer` | `SC.renderer.Start()` returns at once, `IsActive()` is `False`, `DoIdleTasks()` is a no-op |
    | `suppressSolutionViewer` | `mbs.SolutionViewer` and `AnimateModes` return immediately |
    | `suppressPlots` | `PlotSensor` and the other plotting helpers skip `plt.show()`; a figure given a file name is still SAVED |
    | `suppressDialogs` | `InteractiveDialog` and `GUI.py` return their defaults instead of opening a tk window |
    | `SuppressAll(True)` | sets the four |

    A suppressed call is a **silent no-op**, except that each kind prints **one** notice the first
    time it is suppressed - a window-less session must never be a mystery. `IsActive()` returning
    `False` is the load-bearing detail: it is what lets `while SC.renderer.IsActive():` end instead
    of spinning, which is the only reason the example runner rewrites that line today.

    Like its two neighbours the group is a C++ class in `Experimental.h`, so the flags are one
    value read from both sides: the renderer (window and idle loop are C++) and the Python helpers,
    which read `exu.special.userInterface.*`.

    **Environment**: `python/exudyn/__init__.py` reads `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` (sets the
    four flags) and `EXUDYN_OUTPUTDIRECTORY` (sets `exu.config.outputDirectory`, which today can
    only be set from Python). **These would be the first environment variables Exudyn reads at
    runtime** - `getenv` appears nowhere outside `setup.py` - so the step also documents them as
    intended for AI tools and CI, not for users, and prints one line on import when either is
    active. **Both reads are wrapped in `try/except`** and a failure never stops the import
    (maintainer, 2026-09-17): `os.environ` itself is always present, but applying the value is not
    free of risk - `config.outputDirectory` rejects some strings, and a frozen or embedded
    interpreter may hand back something unexpected.

    **How `suppressPlots` reaches scripts that do not use `PlotSensor`** (decided 2026-09-17).
    `PlotSensor` calls `plt.show()` itself (`plot.py:761`), so the package side follows the flag
    directly. But **29 examples and 9 test models call `plt.show()` directly**, having imported
    matplotlib themselves. Those are covered by `matplotlib.use("Agg")`, applied once from the flag
    rather than by editing 38 scripts: one place, and every script written later is covered too.
    Figures are still drawn and still saved under Agg; only the window disappears. The limit worth
    stating: the backend has to be chosen before the first figure is created, so a flag flipped in
    the middle of a script cannot retro-fit it - which is precisely why the environment variable
    exists.

    **What this does NOT replace**: of the ~14 source substitutions in
    `testRunnerTools.ExampleSkipReason`/the example bootstrap, roughly six are window-related and
    can go; the rest cut WORK (`useGraphics = True` -> `False`, `numberOfGenerations`,
    `useMultiProcessing`, `verbose`, `showProgress`) and stay. The step states which are removed.

<a id="r5-17-1"></a>
**R5.17.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-17-1) — *(sub-step of R5.17)*
    **A script that draws its own plots ignored the flag** (#2478): `suppressPlots` reaches every
    plot the package draws, but not the **36 models and examples** that import matplotlib
    themselves and call `plt.show()`. Each now carries a two-line guard after its exudyn import,
    and `CLAUDE.md` rule 11 says that a local run of an existing model sets
    `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` and `EXUDYN_OUTPUTDIRECTORY`.
<a id="r5-18"></a>
**R5.18** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18) — *(phase R5, last step;
maintainer request 2026-09-18)* **One driver instead of sixteen batch files** (#2503).
`tools/buildAndGenerate/` held 18 files - **16 `.bat`**, a README and one shell script - of which
**five existed only to find conda and to loop over the Python versions** (`condaActivate.bat`, `execWithPythonVersion.bat`,
`execWithAllPythonVersions.bat`, plus the two thin wrappers `makeWindowsBinaries.bat` and
`buildInstallSingleVersion.bat`). A `.bat` file cannot print a `--help`, cannot validate an option
and cannot pass an option it does not know about: that is exactly why `--fast-module` - added in
step R5.11 - could not be reached from `runTestSuite.bat` at all until step R5.13.5, and why the
argument-forwarding loop added there needed a dry run to catch that `SHIFT` also shifts `%0`.
The scripts are additionally **where the maintainer looks up how the build works**, a documentation
duty that `REM` headers serve badly.

**The replacement**: one dependency-free Python driver, `python tools/exudev` (a directory with
`__main__.py`, so no `sys.path` manipulation and no installation), with a `.bat` one-liner
`tools/buildAndGenerate/exudev.bat` so that `exudev test --fast` works from any shell. It imports
only `argparse` and `subprocess`, **never `exudyn`**, so it runs under any interpreter - including
the conda base - and can therefore be the thing that *selects* the environment.

| command | what it does | replaces |
|---|---|---|
| `exudev generate [--docs]` | `tools/regenerate.py`, optionally the sphinx build | `runPythonScripts.bat` |
| `exudev build [--py P313] [--fast] [--no-install] [--clean]` | wheel + reinstall for one version or `--py all` | `makeInstallBinaries`, `makeWindowsBinaries`, `buildInstallSingleVersion` |
| `exudev test [--py] [--fast] [--parallel] [--exit-code]` | `runTestSuite.py` | `runTestSuite.bat` |
| `exudev examples [--py]` | `runTestExamples.py` | `runTestExamples.bat` |
| `exudev perf [--py] [--fast]` | `runPerformanceTests.py` | `runPerformanceTests.bat` |
| `exudev docs [--pdf]` | sphinx html; `--pdf` the LaTeX `theDoc` while it still exists | `makeSphinxDoc.bat`, `makeDoc.bat` |
| `exudev linux [--manylinux\|--wsl] [--no-fast]` | the linux wheels through WSL | `makeUbuntuManyLinuxWheels`, `makeUbuntuWheels` |
| `exudev release [--fast] [--no-docs] [--no-tests] [--no-linux]` | the whole path: clean, generate, all wheels, all tests, docs, linux | `makeAndTestAllBinaries.bat` |
| `exudev clean` | build directories and eggs | `removeBuildsAndEggs.bat` |
| `exudev env` | which `venvP3xx` exist and which exudyn version each has | the README warning about stale installs |

**Conventions**, all of them free from `argparse`: `--help` on the driver **and on every
subcommand**; `-q/--quiet`; `--fast` is always **opt-in**; `--docs/--no-docs` and
`--tests/--no-tests` come as a pair from `BooleanOptionalAction`; `--py` takes `P310`...`P314` or
`all`; `--env NAME` overrides the environment (default `venvExuP313` for generation and docs,
`venvP3xx` for the version matrix); **unknown options are forwarded** to the underlying runner, so
a new runner option is usable the day it exists. An unknown *command* is an error with the list of
commands - a `.bat` silently does nothing.

**`-n/--dry-run` is the documentation feature**: it prints the exact command lines it would run and
exits. That answers "how does this actually work" better than the `REM` headers ever did, and it is
how the step is tested.

**Environment selection uses `conda run -n <env> --no-capture-output`** instead of the
activate/deactivate dance. Measured 2026-09-18: 1.8 s overhead per call, exit code propagated,
output not buffered. This removes `condaActivate.bat` and both `execWith*` scripts; the base
installation is still located by `EXUDYN_CONDA_ROOT`, then `CONDA_EXE`, then `PATH`, which is the
logic of `condaActivate.bat` ported to Python.

**Disposition of the 18 files** (the maintainer approved removing all of them, 2026-09-18).
The 16 `.bat` files and the README moved to `tmp/oldScripts/` - `tmp/` is **gitignored**, so this
removes them from the repository. Two corrections to the text above, both found while doing the
work: it said *13* files, and it said `manylinuxBuild.sh` **stays** in the directory. In fact it
moved to **`tools/ci/`**, next to the `buildManylinux.sh` it calls and which already lived there,
and `tools/buildAndGenerate/` is therefore **retired**. `manylinuxBuild.sh` is still shell, because
it runs inside the docker image. The dead `addTags.bat` reference in `makeAndTestAllBinaries.bat`
disappeared with it - no such file exists anywhere in the repository.
`src/pythonGenerator/makeAllBinariesScripts.py` (31 lines, writes `docs/theDoc/buildDate.tex`) is
folded into `exudev release` when that directory is removed.

**The LaTeX `theDoc` is not wrapped** (maintainer 2026-09-18): it has not built for many commits,
paths are wrong and files are missing, and R7 replaces it. `makeDoc.bat` went to `tmp/oldScripts/`
with the rest; no `exudev docs --pdf` was written.

**Deferred on purpose - REMINDER for after the revision**: the **main `README.md`** and the build
instructions in the user documentation still describe the batch files. They are **not** updated in
this step, because R7.2 rewrites the per-platform build instructions anyway and the driver's own
commands may still change; updating both now would mean writing them twice. R7.2 must pick this up,
and [`tools/exudev/README.md`](../../tools/exudev/README.md) is the source it should link to rather
than copy (rule 10). `docs/dev/WORKFLOW.md`, `docs/dev/README.md` and `CLAUDE.md` **were** updated
here, because they describe the developer workflow rather than the user documentation.

<a id="r5-18-1"></a>
**R5.18.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-1) — *(sub-step of
    R5.18)* **The two runners without an exit code**
    (#2504). `runTestSuite.py` returns a real exit code with `--exit-code`; `runTestExamples.py` and
    `runPerformanceTests.py` **always return 0**, however many tests failed. Anything that calls them
    - the driver, a CI job, a shell script - therefore cannot see a failure from the exit code, and
    `tools/exudev/results.py` reads the summary line out of the log instead. That scan cannot tell a
    run that died before writing its summary from a log it did not find, so it answers `unknown` and
    maps it to exit code 2 - honest, but weaker than an exit code. Give both runners the same
    `--exit-code` flag (about 10 lines each; the pattern exists), then **delete `results.py`** and
    let every step of the driver be judged by its return code.

<a id="r5-18-2"></a>
**R5.18.2** *(sub-step of R5.18; found while giving the examples an exit code)* **Three examples
    fail for a missing optional package instead of being skipped** (#2507).
    `testRunnerTools.ExampleSkipReason()` already skips what cannot run - stable-baselines3, rospy,
    a MATLAB peer - but does not cover `numpy-stl` (`humanRobotInteraction.py`,
    `stlFileImport.py`) or `pymeshlab` (`pymeshlabFileImport.py`), so those three are counted as
    failures. They are three of the five entries in `KnownExampleFailures()` today. The decision
    this needs is why it is a step and not three lines: a machine that HAS the package *should* run
    them, so the fix is probably to **try the import** rather than to list file names - and then
    `numpy-stl` and `pymeshlab` belong in the `[all]` extra of `pyproject.toml`, so that a
    developer environment has them.

<a id="r5-18-3"></a>
**R5.18.3** **DONE 2026-09-18** — *(sub-step of R5.18; found by the first GitLab run after the
    driver landed)* **A gate that was green locally and red in CI** (#2508).
    `tools/checkExtras.py` decided which imports are "local" by listing `python/` with `os.listdir`
    and `os.walk`, so **any file present on the development machine** made an import look local.
    `python/pytest.py` - the gitignored scratch copy of `pytestTemplate.py` - did exactly that:
    `import pytest` in `test_testModels.py` resolved to it, the local check said OK, and the GitLab
    job, which has no such file, reported `UNCOVERED IMPORTS: pytest ... needed by [tests]` and
    failed. The same trap applied to any untracked helper dropped into `python/` or `TestModels/`.

    Fixed two ways, because both were wrong: the tool now lists **tracked files only**
    (`git ls-files`), so it sees exactly what CI checks out; and `pytest` has a real exemption entry
    saying what it is - a dev tool declared in `[dependency-groups]`, deliberately not in any extra,
    because the test suite runs without it. Verified by removing the exemption again: the tool then
    prints the CI message word for word, which it could not do before.

## R6 — Error handling and UX (ongoing, after R2)  <!-- old Phase 5 -->

<a id="r6-1"></a>
**R6.1** Audit every bare `except:`; replace with specific exceptions and actionable messages.

<a id="r6-2"></a>
**R6.2** Rewrite binary selection in `__init__.py` as one testable function that logs its decision
    under an env var and raises a single clear `ImportError` listing everything tried. Replace
    the lexicographic `numpy.__version__ <= '2.0'` compare with `packaging.version` or a probe.
    **Hard prerequisite for phase R9.**

<a id="r6-3"></a>
**R6.3** Map `CHECKandTHROW` paths to specific Python exception types; chain `py::error_already_set`
    so user-function tracebacks survive instead of being stringified
    (`ExceptionsTemplates.h:50`).

<a id="r6-4"></a>
**R6.4** Document the error taxonomy.

<a id="r6-5"></a>
**R6.5** **DONE 2026-09-15** — A user switch for parameter range checks. → [log](exudynRevisionLog2026.md#r6-5)

<a id="r6-6"></a>
**R6.6** *(phase R6)* **C++ user errors inspect the Python source** (#2423). `PyError`/`PyWarning` call
    `PyGetCurrentFileInformation` (`src/Main/Stdoutput.cpp:259`), which calls
    `inspect.getframeinfo`; that scans `sys.modules` and reads the source file. The ~38000 probe
    errors of `parameterConversionTest.py` took 1 s standalone and 9 s inside `runTestSuite.py`
    after scipy, matplotlib and ngsolve were imported - a cost wherever errors are caught in a loop.
    The frame itself (`f_code.co_filename`, `f_lineno`) carries the same information.

<a id="r6-7"></a>
**R6.7** *(after R4.4.3.6, before the error-message work of phase R6)* **One exception type per kind of
    parameter error** (#2432). Today a wrong parameter value raises one of three types, depending on
    the path:
    - `RuntimeError` from `PyError` or a pybind11 `cast_error`;
    - `TypeError` from a pybind11 signature mismatch;
    - `ValueError` from the former Python checks.

    R4.4.3.4/34c5 moved most paths to `RuntimeError`. **Decision (2026-09-15):** correct this throughout
    the revision at one step. For example, `TypeError` for a wrong type (string, list, `None`, an item
    index into a scalar) and `ValueError` for a range violation or a wrong size, raised from
    `PyConversion.h`. Applied as its own reference update of `parameterConversionTest.py`.

## R7 — Documentation (~3 weeks)  <!-- old Phase 6 -->

<a id="r7-1"></a>
**R7.1** Sphinx (readthedocs) stays; the sources become **MyST Markdown** (`myst-parser`, dev-only):
    hand-written chapters are converted from `.tex`, generated reference pages come from the docs
    emitters as `.md`, remaining `.rst` files are converted as they are touched; new documentation
    is Markdown from now on (decision 2026-09-15). The PDF is generated via `latexpdf` (front page
    through `latex_elements`). Deletes `latexConverter.py` and `doc2rst.py`.

    **Python docstrings become Markdown too, early rather than late** (can run before step R7.1):
    list the LaTeX in `#**`-style comments by searching for `\` (expected: `$` math and
    abbreviation macros only) and replace it by plain Markdown.

    **Known:** the LaTeX PDF build currently fails (files missing or changed); this surfaces here.

    **Absorbs R4.1.3, the documentation format of the definitions** (`definitions/`): item and
    structure descriptions carry LaTeX macros today (`\hac{ODE2}`, `$\Jm_P$`); the format they
    move to is decided together with this step's converter and macro decision.

    **Include the Markdown documentation in the published build.** `docs/dev/*.md`
    (`ARCHITECTURE`, `CODING_STYLE`, `WORKFLOW`, `README`) and the surviving `docs/howTo/*.md` are
    being written *now* on the assumption they become visible on readthedocs — so Sphinx needs
    `myst-parser` (or equivalent) and toctree entries for them. Without that they stay
    repository-only files that nobody outside the clone ever reads, which defeats the point of
    writing them as documentation rather than as notes.

    This also resolves the duplication `CODING_STYLE.md` currently warns about: once Markdown is
    published, the LaTeX copies of the coding rules and the C++ structure can be deleted rather
    than kept in sync. `introduction.tex` already points at `ARCHITECTURE.md` instead of the
    removed doxygen (step R3.10).

    Note that the .tex files were the original sources and .rst files do not contain all 
    information, which means that the .tex files in docs/theDoc, which are not auto-generated, 
    need to be converted "manually" to a .md version. 
    The PDF had a specific front page which could be kept similarly in the final 
    generated PDF and the TOC in the PDF was good to get an impression of the material contained -
    questioning if any of this can survive in the pdf compiled from .md.

    After latex has been abandoned, as well as the doc2rst.py (check if there is something that 
    will be still needed from the old latex converters, like the abbreviation list at the end of
    doc2rst, etc.). Further, the autoGenerateHelper.py - which is in a terrible state, probably 
    most terrible in the project - will not require most of its functions, so cleanup is needed.

<a id="r7-1-1"></a>
**R7.1.1** *(sub-step of R7.1; maintainer question 2026-09-17)* **What happens to the tikz figures.**
    Measured before answering: `introduction.tex`, `solver.tex` and `theory.tex` contain **13**
    `tikzpicture` environments, and each already has a twin - the `.tex` sources carry **both**
    representations side by side:

    ```latex
    \onlyRST{ .. figure:: docs/theDoc/figures/solversAvailableSolvers.png }
    \ignoreRST{ \begin{figure} \begin{tikzpicture} ... \end{tikzpicture} \end{figure} }
    ```

    So the HTML documentation has **never** rendered tikz: it shows a hand-made PNG of it, and the
    two are kept in step by hand (15 such `figure::` blocks). Markdown therefore loses nothing that
    Sphinx has today - but it is the moment to end the duplicate, which is exactly what rule 10 of
    `CLAUDE.md` warns about.

    All 13 are node-and-arrow flowcharts, not geometry. The recommendation is therefore
    **mermaid**: it is text, so it diffs and reviews like code; MyST renders it natively and so does
    GitHub; and it removes the second copy. For the PDF, mermaid is pre-rendered to SVG at build
    time (`mermaid-cli`, dev-only) - or, if the PDF's typography must stay tikz, then tikz becomes
    the single source and the PNG is **generated** (tikz -> pdf -> svg) instead of hand-made, which
    costs a LaTeX toolchain in the documentation build. Either way the hand-maintained twin ends.
    Genuinely geometric figures, if any appear, keep a pre-rendered SVG.

<a id="r7-1-2"></a>
**R7.1.2** *(sub-step of R7.1; noticed 2026-09-18)* **A generated page is in no toctree** (#2505).
    The sphinx build prints `docs/RST/TestModels/sphereTriangleTest.rst: WARNING: document is not
    included in any toctree` - the page is generated but unreachable, findable only by search. The
    build is not run with `-W` here, so nothing fails today; the check is whether the generator that
    writes the TestModels pages also writes their index entry, in which case this is one missing
    entry rather than one missing page.

<a id="r7-2"></a>
**R7.2** *(after R7.1, when the documentation is Markdown)* **Carry the revision into the documentation.**
    Extract every change recorded in this plan and in `exudynRevisionLog2026.md` - new flags and
    switches (e.g. `exudyn.special.exceptions.parameterRangeChecks`), conversion and error behaviour,
    definitions and generators, howto build on each platform (put the simple way also into the main README), tools and workflow - and update the user and developer documentation accordingly. The plan and log are records, not documentation; afterwards the plan is reduced to an archive.

<a id="r7-3"></a>
**R7.3** Stop committing generated RST and `theDoc.pdf`; build in CI, publish the PDF as a release
    asset.

<a id="r7-4"></a>
**R7.4** Convert `trackerlog.tex` into `CHANGELOG.md`.

<a id="r7-5"></a>
**R7.5** *(phase R7)* **Rewrite the installation documentation** (#2388). `gettingStarted.tex` still
    describes Python 3.6/3.7, 32-bit Anaconda and wheel names from 2020; rewrite against what is
    shipped (cp310-cp314, 64-bit only). With step R7.1 this moves to the RST side.

## R8 — Process  <!-- old Phase 7 -->

<a id="r8-1"></a>
**R8.1** Issue and PR templates, and a `CONTRIBUTING.md` stating the actual policy now that a branch
    exists to target.

<a id="r8-2"></a>
**R8.2** `tools/release.py`: bump → regenerate → test → build → tag.

<a id="r8-3"></a>
**R8.3** *(phase R8)* **Give `issueTracker.py` a CLI.** Today it is driven by importing the module and
    calling functions from its own directory. Add argparse — `raise`, `resolve`, `list`, `show`,
    `modify`, and **`--release` / `--dev` to switch the build mode** (fact 26), which is currently a
    hand edit of `versionDev` at line 56-57 followed by `UpdateFiles()`. A small Tkinter front-end
    over the same commands is wanted afterwards. Remove the cwd dependency and the hard-coded
    Windows path separators
    (`'..\\..\\main\\src\\Autogenerated\\'`), and add tests around
    `ResolvedIssues2Version`/`GetMajorMinorMicroVersion`, which no test covers today despite being
    the version's only definition. Fix the known inconsistencies listed in `docs/dev/WORKFLOW.md`
    while there: the `NORMAL`-vs-`med` priority mismatch and the stale
    `cd ..\tools\makeWindowsBinaries\` in `execWithPythonVersion.bat`. A web mask can follow later;
    the CLI is the part that unblocks scripting and CI.

<a id="r8-3-1"></a>
**R8.3.1** *(sub-step of R8.3; maintainer request 2026-09-17)* **An issue can also be closed
    WITHOUT being resolved.** The tracker knows `RAISED`, `WORK` and `RESOLVED`; there is no way to
    record "decided against", "no longer applies" or "superseded", so such issues either stay open
    forever - the 2016-2020 backlog is full of them - or get marked RESOLVED, which is untrue and
    also **changes the version number**, because the micro version is derived from the count of
    resolved issues (rule 2). That is the constraint this step has to respect: a closed-not-fixed
    issue must NOT count as resolved.

    Add one status - suggested `CLOSED`, with a mandatory reason in the notes, rather than the
    Bugzilla-style pair `WONTFIX`/`OBSOLETE`, because the distinction is prose and every extra
    status is another branch in the converters. Touches: the status list and `ResolvedIssues2Version`
    in `issueTracker.py`, the RST/LaTeX/HTML converters, the CLI of R8.3 (`close <n> --reason ...`),
    and the JSON schema of R8.5. Also check `ChangeIssue` cannot set it silently.

<a id="r8-3-2"></a>
**R8.3.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r8-3-2) — *(sub-step of R8.3;
    maintainer request 2026-09-17)* **The old backlog was checked against what the revision actually
    did.** 29 issues from 2016-2026 were read and verified one by one; 23 are resolved by later
    work, 2 were partially covered and got successors (#2497, #2498), 6 stay open because nothing
    has been done about them.

<a id="r8-4"></a>
**R8.4** *(phase R8)* **Fold minor-version bumps into the tracker.** A 1.11 → 1.12 bump currently means
    hand-editing the `versionResolved` list and `versionNames` dict inside `issueTracker.py`. Make
    it a command that records the baseline automatically. Keep it an explicit maintainer action,
    never automatic.

<a id="r8-5"></a>
**R8.5** *(phase R8, after R8.3)* **Migrate `trackerlog.txt` to one file per issue, in JSON.**
    This is the intended end state: reviewable diffs, no comma-escaping trap (`\;`), no single-file
    merge conflicts. **JSON, not Markdown**, so that the format cannot drift and import/export stay
    trivial.

    **Layout**:

    ```
    docs/dev/issues/
      open/2475.json            one file per OPEN issue
      resolved/2473.json        one file per issue resolved since the cutoff
      archive/2019.json ...     one file per YEAR, frozen once written
    ```

    The cutoff is 1 January of the previous year (today: before 2025-01-01). Archiving is an
    explicit maintenance command, not a side effect of `ResolveIssue`, so the move is one
    reviewable commit. **The migration writes the archives directly**: the 2400 single files must
    never exist, not even for one commit, or the repository carries them in its history forever.
    Sharding the archive by year - rather than one file - keeps each one write-once: no rewrite of
    a large file on every archiving run, and nothing to merge.

    **Fields to add while the format is being defined** (cheap now, painful to retrofit):
    `schemaVersion`; `status` as an enum including `duplicate` with `duplicateOf`;
    `resolvedInVersion` - the version the resolution produced, which makes R7.4's `CHANGELOG.md`
    a pure rendering job instead of a second mechanism; `planStep` as a real field (today
    "revision2026 step R5.4.5" is prose inside the notes); `component` (solver / linalg / python /
    build / docs) for filtering; `resolvedCommit`, the hash. And **fix `priority`**: #2388, #2398
    and #2400 print "priority undefined" on every tracker call today.

    **The one hard coupling**: the micro version is derived from the **count of resolved issues**.
    With files, that count depends on the working tree being complete - a partial checkout would
    silently LOWER the version. Each archive file therefore carries its own `resolvedCount`, and
    the validator checks the total against the files it can see and fails loudly on a mismatch.

    **Validation in the commit gate**: a `--check` in `tools/checkAll.py` - no duplicate ids, valid
    enum values, required fields present, counts consistent. No new dependency (a small validator,
    not `jsonschema`).

    **Lossless migration, proven**: export to JSON, regenerate `trackerlog.txt` from the JSON and
    byte-compare against the current file. When it lands, the old path is **deleted**, not kept in
    parallel - otherwise the escaping trap survives.

    **Generated artifacts are not committed** (the HTML overview, any generated index), with **one
    deliberate exception**: the rendered list the documentation shows. `docs/RST/trackerlog.rst`
    stays committed and stays generated, because that is what ReadTheDocs renders (maintainer,
    2026-09-17). `docs/theDoc/trackerlog.tex` needs no decision here - it disappears with the
    LaTeX documentation.

<a id="r8-5-1"></a>
**R8.5.1** *(sub-step of R8.5)* **A tiny local viewer/editor for the issues**, for maintainers:
    `python tools/issueTracker/serve.py` opens a local web page - **stdlib `http.server` and one
    HTML page, no new dependency** (a Qt6 front-end would cost PySide6, against rule 6, and a web
    page also works over SSH). Features: list by id, search, filter open / resolved / both, and
    **RaiseIssue, EditIssue, ResolveIssue** writing through the same API the scripts use.
    **Deleting an issue stays manual and file-based** - it should be rare (a wrongly raised issue)
    and deliberate.

<a id="r8-6"></a>
**R8.6** *(phase R8, last step of this plan; maintainer request 2026-09-15)* **Checker for user scripts
    after the v2.0 API changes.** Teaching folders and user projects hold Exudyn scripts written
    against 1.x. A static checker (parses, never runs) reports per file and line: names the script
    uses but no longer gets from a star import (`np`, `sin`, `graphics`, ...; step R4.22.3), removed
    names with their replacement (R4.22.1, R4.22.2), and submodules used without their import, with the
    import line to add. The name lists come from the modules' `__all__` and from the table
    [API changes for the v2.0 release notes](exudynRevisionInfo2026.md#api-changes-v2), which is
    complete only once the other steps are done - hence last. Decide then whether it ships in the
    package (users run it) or stays in `tools/`. The plan continues in a new document after this step.

## R9 — Compiled user extensions (after R6)  <!-- old Phase 8 -->

Additive; nothing earlier changes. Two shipped variants remain after step R2.10, so a plugin is
bound to the one it was built against (`use_AVX2` changes `exuMemoryAlignment`); step R9.2 turns
that from silent corruption into a refusal.

**Founding policy**: a plugin author builds exudyn from source once, then builds the plugin with
the same toolchain. Compiler, CRT and flags are then identical by construction, so no C-ABI shim
is needed — plugins inherit from `CObject` directly.

<a id="r9-1"></a>
**R9.1** Make the registry cross-binary. `MainObjectFactory.h:97` holds its singleton in a
    function-local static inside a header-only class template, so every binary gets a private
    copy. Move the storage into one exported accessor in a single translation unit. The dispatch
    path needs no change.

<a id="r9-2"></a>
**R9.2** Define an ABI fingerprint checked at registration: exudyn version, active macro set, compiler
    id and version, `__cplusplus`, and on MSVC `_ITERATOR_DEBUG_LEVEL` plus the CRT model. Add
    `sizeof` canaries for `Vector`, `CObject`, `std::string`, `std::function`. Refuse with a
    message naming the mismatch. **The Debug/Release CRT case matters most** — the registry holds
    `std::map<std::string, std::function<...>>`, and debugging a new object in VS is exactly what
    a plugin author will do.

<a id="r9-3"></a>
**R9.3** Commit a reference plugin subdirectory built on every commit: minimum compile, a trivial
    registered object, a handshake assertion, and a test that adds it to a system and solves.
    **This is the synchronisation mechanism** — interface drift breaks your build, not a user's.

<a id="r9-4"></a>
**R9.4** Ship the plugin headers in the wheel; add `exudyn.get_include()` (numpy/pybind11 convention).
    Headers only: `Linalg/`, `Utilities/`, item base classes, plugin interface header.

<a id="r9-5"></a>
**R9.5** Change duplicate handling in `RegisterClass` from `CHECKandTHROWstring` to a collected,
    reported error. Otherwise two third parties choosing the same name abort `import exudyn`.

<a id="r9-6"></a>
**R9.6** Discovery at import: `~/.exudyn/plugins/` plus `EXUDYN_PLUGIN_PATH`, and
    `importlib.metadata` entry points for pip-installed plugins. Read a manifest *before* loading
    the library. Load each in isolation; a broken plugin must never break `import exudyn`. Never
    put the plugin directory inside the installed package.

<a id="r9-7"></a>
**R9.7** Record loaded plugins and versions in the solver log and solution file header. A script
    yielding different results on a colleague's machine because of a library in their home
    directory is a reproducibility hazard.

<a id="r9-8"></a>
**R9.8** Extend the step R4.3 emitter to scaffold a plugin from a user's definition file — C++ skeleton,
    Python dict-building class, `.pyi` stub — emitted into the user's package.

<a id="r9-9"></a>
**R9.9** Document three constraints: plugins are never unloaded or reloaded (a rebuilt plugin needs a
    kernel restart in Spyder/Jupyter); plugin authors build from source; the Python
    dict-builder class stays an explicit `from myplugin import ObjectMyThing` rather than being
    injected into `exudyn.itemInterface`, so every script says where its item types came from.

## R10 — Deeper implementation problems (last)  <!-- old Phase 9 -->

A holding phase for problems that are real, reproducible, and too deep to fix while the
restructuring is in flight. They are recorded here rather than worked around silently, so the
debt stays visible and each item can be closed on evidence.

<a id="r10-1"></a>
**R10.1** **Resolve the Windows/Linux differences in contact and friction models.** Measured 2026-09-10
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

<a id="r10-2"></a>
**R10.2** *(phase R10)* **`ObjectContactConvexRoll.pContact` becomes a data variable** (#2413). The
    computed contact point is stored in the parameter structure and read by the visualization, so
    it is neither system state nor configuration-dependent and keeps no history.

<a id="r10-3"></a>
**R10.3** *(phase R10)* **Explicit integration cost** (#2398, #2400). With the default dense linear solver
    an explicit step on a chain of point masses costs O(N^2) (168 ms per step at N=2000; 400 times
    faster with `EigenSparse`), and `computeMassMatrixInversePerBody` changes nothing unless a
    sparse solver is selected as well. At least warn at large N; better, avoid the global solve
    in explicit integration where the flag makes it unnecessary.

<a id="r10-4"></a>
**R10.4** **DONE 2026-09-17** (verified, not worked on) — *(phase R10 candidate)*
    **`ObjectANCFThinPlate` added with its defaults fails inside C++** (#2430). It does not any
    more: checked on 1.11.135.dev1, `mbs.AddObject(ObjectANCFThinPlate())` adds with its defaults
    exactly as `ObjectMassPoint` and `ObjectANCFCable2D` do - which is what the issue named as the
    expected behaviour. Fixed on the way by the item-interface work of step R4.4.3.

## R11 — Misc (came up during the revision)

<a id="r11-1"></a>
**R11.1** *(before the rendering revision)* **Remove OpenVR.** It **blocks the rendering revision**, it
    is not testable in CI or by most users, and it carries a vendored SDK and a prebuilt binary.
    Scope: `main/src/Graphics/OpenVRinterface.cpp` and its header, every `__EXUDYN_USE_OPENVR`
    guard, the `--openvr` flag and `-lopenvr_api` in `setup.py`, `main/include/openVR/`, and
    `main/libs/openvr_api.dll` + `.lib`. Users needing OpenVR take Exudyn <= 1.11; say so in the
    release notes rather than leaving them to discover it. `docs/howTo/openVR.txt` was already
    removed with step R3.6.

<a id="r11-2"></a>
**R11.2** *(before R11.3; maintainer decision 2026-09-16; revised 2026-09-17)* **A maintained
    micro-benchmark for the linear algebra, inside Exudyn** (#2397, from step R2.16).

    *The starting point named by the original text is gone.* This step used to say "replace the
    dead sweep in `PyTest()` (`src/Pymodules/pythonTests.cpp`)"; that file was **deleted** in step
    R5.4.11 (#2484) as outdated and misleading. Nothing is replaced, then - this step **writes** the
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

    The pattern to follow is `src/Linalg/symbolicCppDemo.h` from step R5.4.12: named functions, one
    topic each, compiled by being included, and a header that says what it is for.

<a id="r11-3"></a>
**R11.3** *(after R11.2)* **Make the hot linear algebra vectorizable.** Step R2.16 measured that the
    solver time of long-vector models sits in `ODE2RHS` (73-91 % of the explicit runs), i.e. in
    per-object 3x3 and short-vector work, not in the long-vector loops that AVX2 accelerates, and
    `ConstSizeMatrix` carries its size at runtime, so the compiler cannot unroll it. Candidates:
    compile-time sizes where the size is known, more use of homogeneous transformations in the
    rigid-body kinematics, and the object loop of `ODE2RHS` itself. Steered by the benchmark of
    R11.2; a compile-flag decision alone (step R2.16) cannot achieve this.
