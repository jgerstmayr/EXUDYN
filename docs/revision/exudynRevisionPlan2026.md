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
**R2.10.4** *(sub-step of R2.10)* **`FEMinterface` NPZ files cannot be read by a second module**
    (#2471): `SaveToFile(mode=NPZ)` stores `postProcessingModes['outputVariableType']` as an
    `exudyn.exudynCPP.OutputVariableType`, so `np.load(allow_pickle=True)` imports that module and
    a process holding `exudynCPPfast` gets `type "Real" is already registered`. Every other field
    loads under both modules. Store the enum by name and convert back on load; then
    `NGsolveCMStest` can leave `NotJudgedOutsideRegularModule()`.


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
**R5.4.2** *(sub-step of R5.4)* **Symbolic**: `Symbolic.h`, `SymbolicVector.h`, `SymbolicMatrix.h`.
    ~2500 lines with a Python-level test already in place (`symbolicModuleTest.py`,
    `symbolicUserFunctionTest.py`), so the C++ tests should aim at what Python cannot reach: the
    expression tree itself, `Diff`, and evaluation after a variable changes.

<a id="r5-4-3"></a>
**R5.4.3** *(sub-step of R5.4)* **`LinearSolver.h`**: the dense and the sparse (Eigen) solver
    behind one interface — the same shape as `MatrixContainer` in R5.4.1, and the same kind of test:
    both must answer identically for a system that is solvable, and both must fail recognisably for
    one that is not.

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
**R5.4.6** *(sub-step of R5.4, from R5.4.5)* **`SparseTripletMatrix(rows, columns, triplets)`
    throws its size arguments away** (#2476): it initialises both to 0 and never assigns the
    arguments, so the matrix reports 0 x 0 while holding the triplets. Nothing calls it today, but
    #2474 made the size fields load-bearing. Assign them or delete the constructor.
<a id="r5-5"></a>
**R5.5** *(phase R5, tooling)* **A linter and a type check for the Python side.** Two separate
    halves, neither started:

    - **ruff** over the shipped package `python/exudyn/`. Nothing in the repository configures it
      today; the only linter in use is `pydoclint`, whose findings are frozen in
      `tools/ci/pydoclintBaseline.txt`. **To decide**: which rule set (the default E/F, or more),
      whether the existing findings get a baseline like pydoclint's or are fixed outright, and
      whether it runs in the commit gate or only in CI.
    - **a type check whose purpose is the stubs**: `python/exudyn/__init__.pyi` and
      `python/exudyn/symbolic.pyi` describe the C++ bindings and are merged at build time by
      `tools/generators/createStubFiles.py`. Nothing verifies that they still match what the module
      exports, so a renamed binding leaves a stub that lies to every IDE. mypy or pyright can be
      pointed at exactly this. **To decide**: which checker, and whether the run covers the package
      as a whole or only stub-vs-module agreement.
<a id="r5-6"></a>
**R5.6** Add an ASan/UBSan Linux job. For a C++ library invoking arbitrary user callbacks this catches
    the class of bug users report as "it crashed with no message".

<a id="r5-7"></a>
**R5.7** **DONE** — rename `pytest.py` - done differently in step R3.1 (`python/pytestTemplate.py`). → [log](exudynRevisionLog2026.md#r5-7)

<a id="r5-8"></a>
**R5.8** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-8) — *(phase R5)* **`runTestSuite.py --parallel[=N]`**: every model in its own interpreter, 22 s → 9-11 s; serial stays the default for the commit gate.

<a id="r5-9"></a>
**R5.9** **DONE 2026-09-11** — Complete and verify the test list. → [log](exudynRevisionLog2026.md#r5-9)

<a id="r5-10"></a>
**R5.10** **DONE 2026-09-10** — `testRunnerTools.ResolveLogFile()` decides the log target before the first write. → [log](exudynRevisionLog2026.md#r5-10)

<a id="r5-11"></a>
**R5.11** *(phase R5, release testing)* **Cover every compiled variant in the release tests.** Windows
    release builds produce **three** modules — `exudynCPP`, `exudynCPPfast`
    (`__FAST_EXUDYN_LINALG`) and `exudynCPPnoAVX` — and the suite exercises only whichever one
    `__init__.py` selects. The fast and noAVX binaries therefore ship essentially untested, which
    matters more after step R2.10 consolidates to two shipped variants selected by a CPUID check.

    Selection is already scriptable: `__init__.py:35-42` reads `sys.exudynFast` and
    `sys.exudynCPUhasAVX2` *before* the C++ module is imported, and `runTestSuite.py` imports `sys`
    at line 15 but `exudyn` only at line 33 — so `-fast` / `-noavx` options can set them. Verified
    2026-09-09 that this loads exactly one binary: with `sys.exudynFast=True`, `sys.modules` holds
    `exudyn.exudynCPPfast` and no `exudyn.exudynCPP`.

    The blocking obstacle is already removed: `runTestSuite.py` used to `import exudyn.exudynCPP`
    unconditionally just to report the binary path and build date, which would have pulled the
    default binary into the process alongside the intended one and then reported the wrong module
    as the one under test. It now resolves whichever module `sys.modules` actually holds.

    What remains: the `-fast` / `-noavx` options themselves, and a release procedure that runs all
    three and keeps all three logs. Sequence after step R6.2 (rewriting binary selection) if that
    lands first — the two touch the same logic. After step R2.10 there are **two** variants, not
    three, and `-noavx` is gone with the module it selected.

<a id="r5-11-1"></a>
**R5.11.1** *(sub-step of R5.11; maintainer decision 2026-09-16)* **How much of the matrix the fast
    variant needs.** The fast module is the same source compiled with two macros, so what can break
    in it is compiling, loading and the numerical effect of AVX2 — not Python-version behaviour.
    Therefore:

    | what | against which Python versions |
    |---|---|
    | full `runTestSuite.py`, default module | every supported version (unchanged) |
    | full `runTestSuite.py`, fast module | the **oldest** and the **second newest**, today 3.10 and 3.13 |
    | examples | one version (unchanged) |
    | `runPerformanceTests.py` | fast module, plus one default-module run for comparison |

    The newest version (today 3.14) is deliberately **not** the fast-mode target: right after a
    release its packages are the unstable part, so a failure there would almost never be about the
    fast module. Today `exudynCPPfast` is built only for Python 3.10 in development versions
    (`setup.py`) and only `runPerformanceTests.py` ever sets `sys.exudynFast`, so the fast binary
    ships untested — this sub-step is what makes the second variant of step R2.10 affordable
    *and* covered.

<a id="r5-12"></a>
**R5.12** *(phase R5, small)* **Test and example hygiene** (#2368, #2377). `ANCFbeltDrive.py` yields 0.0
    against its recorded reference -0.484 since it was retuned to a 10 s run - find which is
    right before it enters the suite (#2368). Three imports name modules that exist nowhere or
    only by accident of `sys.path` (`RL_Spot`, `timeIntegrationOfRotationVectorFormulas`, a bare
    `rosInterface`); fix or remove them and shrink `knownMissingLocalModules` in
    `tools/checkExtras.py` accordingly (#2377).

<a id="r5-13"></a>
**R5.13** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-13) — *(phase R5, with R5.8 and R5.9)* **Test-suite output goes to its own directory** (#2418, #2454): `exudyn.config.outputDirectory` and one output directory per model; no model writes next to itself any more.

<a id="r5-13-1"></a>
**R5.13.1** *(sub-step of R5.13)* **Stop writing what nothing reads.** Step R5.13 removed the
    collisions; the writing itself is still there: 24 models set `writeSolutionToFile=True` and
    32 write sensor files, although the values are read back only by `compareFullModifiedNewton`
    (the designated writing test) and, under `useGraphics`, by 11 models that plot from the files.
    Convert those to `storeInternal=True` with `mbs.PlotSensor`, and set `writeSolutionToFile=False`
    where the file is never read. Also: `NGsolveCMStest`, `abaqusImportTest` and `pickleCopyMbs`
    write generated meshes into the **tracked** `testData/` input directory
    (`netgenTestMesh2.hdf5/.pkl`, `netgenTestMesh22.npz`), and `geneticOptimizationTest`,
    `pickleCopyMbs` and the FEM tests write `.npz`/`.pkl`/`.h5` files through plain Python calls,
    which `exudyn.config.outputDirectory` does not reach - these need the path in the model.

<a id="r5-13-2"></a>
**R5.13.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-2) — *(sub-step of R5.13)*
    **Five examples wrote sensor output next to themselves** (#2475): `beltDriveALE` and
    `beltDriveReevingSystem` into `solutionDelete/`, the latter also `solution_nosync/`,
    `rigidBodyIMUtest` into `solutionIMU<mode>/`, and the two `sliderCrank3DwithANCFbeltDrive`
    examples into the current directory plus `plots/`. Only `solution/` is ignored, so each direct
    run left untracked, unignored files behind. All of it now goes through `solution/`, and the
    reads through `OutputFilePath`.

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

<a id="r7-2"></a>
**R7.2** *(after R7.1, when the documentation is Markdown)* **Carry the revision into the documentation.**
    Extract every change recorded in this plan and in `exudynRevisionLog2026.md` - new flags and
    switches (e.g. `exudyn.special.exceptions.parameterRangeChecks`), conversion and error behaviour,
    definitions and generators, tools and workflow - and update the user and developer
    documentation accordingly. The plan and log are records, not documentation; afterwards the plan
    is reduced to an archive.

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
**R10.4** *(phase R10 candidate)* **`ObjectANCFThinPlate` added with its defaults fails inside C++**
    (#2430). `mbs.AddObject(ObjectANCFThinPlate())` raises `ResizableArray<T>::operator[], i < 0`
    even with range checks off: the four `InvalidIndex` node numbers are used while the object is
    added. Every other item class either adds with its defaults or names the parameter that must be
    given (R4.4.3.4e. Expected: a message naming `ObjectANCFThinPlate.nodeNumbers`, or
    `CheckPreAssembleConsistency` catching it, with no index access during Add.

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
**R11.2** *(before R11.3; maintainer decision 2026-09-16)* **A maintained micro-benchmark for the
    linear algebra, inside Exudyn** (#2397, from step R2.16). The sweep that would answer "does
    vectorization pay" is dead code in `PyTest()` (`src/Pymodules/pythonTests.cpp`): it is inside
    comment blocks and `if (0)`, `exu.Test()` is not even bound in a release build
    (`EXUDYN_RELEASE`), and it timed hand-written loops rather than the vector code the solver uses.
    Replace it by a benchmark that is compiled into every build and runs the **real** operations -
    `Vector`/`ResizableVectorParallel` add, subtract, scale and `MultAdd`, `SlimVector`/`Matrix3D`
    products, `ConstSizeMatrix` and matrix-vector products - over a size sweep that crosses
    `ResizableVectorParallelThreadingLimit`, single- and multithreaded. Exposed as
    `exudyn.special.RunLinalgBenchmark()` (flags for sizes, repeats, which groups) so the user
    interface grows by one function; `exu.special` already exists (`PySpecial` in
    `src/Main/Experimental.h`, bound from `definitions/pybindModule.py`). This also lets a user
    measure their own CPU: which build flags and how many threads make sense there. The tool-level
    benchmark `tools/benchmarks/avx2Benchmark.py` stays as the solver-level counterpart.

<a id="r11-3"></a>
**R11.3** *(after R11.2)* **Make the hot linear algebra vectorizable.** Step R2.16 measured that the
    solver time of long-vector models sits in `ODE2RHS` (73-91 % of the explicit runs), i.e. in
    per-object 3x3 and short-vector work, not in the long-vector loops that AVX2 accelerates, and
    `ConstSizeMatrix` carries its size at runtime, so the compiler cannot unroll it. Candidates:
    compile-time sizes where the size is known, more use of homogeneous transformations in the
    rigid-body kinematics, and the object loop of `ODE2RHS` itself. Steered by the benchmark of
    R11.2; a compile-flag decision alone (step R2.16) cannot achieve this.
