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
**R2.10** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-10) — **Two shipped variants, one meaning on every platform** (#2466).

<a id="r2-10-1"></a>
**R2.10.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-1) — *(sub-step of R2.10)* **`exudynCPPfast` no longer segfaults** (#2467).

<a id="r2-10-2"></a>
**R2.10.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-2) — *(sub-step of R2.10)* **`NGsolveCMStest` no longer rewrites its own committed input** (#2469).

<a id="r2-10-3"></a>
**R2.10.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-3) — *(sub-step of R2.10)* **A second reference set for the AVX2 module**

<a id="r2-10-4"></a>
**R2.10.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-4) — *(sub-step of R2.10)* **`FEMinterface` files can be read by any module** (#2471).

<a id="r2-10-5"></a>
**R2.10.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r2-10-5) — *(sub-step of R2.10; maintainer request 2026-09-17)* **The platform string names the architecture** (#2499).

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
**R2.17** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r2-17) — *(phase R2 tooling, before the next header-only change)* **The wheel build does not see header changes** (#2427).

<a id="r2-17-1"></a>
**R2.17.1** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r2-17-1) — *(sub-step of R2.17)* **A compile-FLAG change now discards the previous build** (#2468).

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
**R3.8** **DONE 2026-09-18** — the four log directories consolidated into `python/logs/`
    (`testmodels`, `examples`, `performance`, `tmp`). The step originally said a **top-level**
    `logs/`; the maintainer chose `python/` instead, so that the repository root stays the code
    and how to build it, and everything a test run touches is in one subtree (#2512).
    → [log](exudynRevisionLog2026.md#r3-8)

<a id="r3-9"></a>
**R3.9** **DONE 2026-09-18** — `python/TestModels/` holds 127 test models and nothing else; the
    runners are in `python/testing/`, the performance models in `python/PerformanceModels/`, the
    generated mini examples in `python/MiniExamples/`. `NotTestModels()` is gone and the
    performance suite has a coverage check of its own. The one dual-use model was split into two
    copies with the switching removed (#2513). → [log](exudynRevisionLog2026.md#r3-9)

<a id="r3-9-1"></a>
**R3.9.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r3-9-1) — *(sub-step of R3.9, found by the gates of the next step)* **`checkExtras.py` did not know `python/testing/`** (#2514).

<a id="r3-11"></a>
**R3.11** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r3-11) — *(new step, on the maintainer's
    instruction 2026-09-18)*
    **Every tracked text file is UTF-8** (#2533), and `tools/checkEncoding.py` keeps it that way
    in `exudev generate --all-checks`. R0.6 made the **generators** explicit about `utf-8`; the
    **sources** were never converted.

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

<a id="r4-3-1"></a>
**R4.3.1** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r4-3-1) — *(sub-step of R4.3)*
    **The emitter writes its five outputs again** (#2526), and `generate.py` now **fails** a
    stage that exits 0 without producing any of its declared outputs — the part that matters
    beyond the one file.


<a id="r4-4"></a>
**R4.4** **DONE 2026-09-15** — emitters read members directly (R4.4.1); one Python/C++ conversion layer `PyConversion.h` (R4.4.3); Jinja2 measured and not adopted (R4.4.2). → [log](exudynRevisionLog2026.md#r4-4)

<a id="r4-5"></a>
**R4.5** **DONE 2026-09-15** — MainSystem extensions bound by `@extends(exudyn.MainSystem)` and `install()` instead of copy-and-append (#2434). → [log](exudynRevisionLog2026.md#r4-5)

<a id="r4-6"></a>
**R4.6** **DONE 2026-09-15 (R4.6.1-R4.6.4, with 37).** → [log](exudynRevisionLog2026.md#r4-6) — Migrate the `#` convention to Google-style docstrings across all 27 utility modules.

<a id="r4-7"></a>
**R4.7** **DONE 2026-09-15 with R4.6.2/36c** — `@docmeta(author, date, status, public)` in `exudyn/docmeta.py`. → [log](exudynRevisionLog2026.md#r4-6)

<a id="r4-8"></a>
**R4.8** **DONE 2026-09-15** — `pydoclint` check of `python/exudyn` in GitLab CI, with a baseline (#2439). → [log](exudynRevisionLog2026.md#r4-8)

<a id="r4-9"></a>
**R4.9** **DONE 2026-09-15** — install-time docstring converter and `autoGenerateDocstrings.py` removed (#2439). → [log](exudynRevisionLog2026.md#r4-8)

<a id="r4-10"></a>
**R4.10** **DONE 2026-09-15 (R4.10.1-R4.10.4)** → [log](exudynRevisionLog2026.md#r4-10) — *(phase R4, after R4.3)* **Expose the type information to Python** (issue #2411; was R4.1.6).

<a id="r4-11"></a>
**R4.11** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-11) — *(phase R4, before R4.3, small)* **Generator correctness** (#2414, #2415).

<a id="r4-12"></a>
**R4.12** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-12) — ; done as a comment, not as typedefs (`PReal` is the AVX packed-real macro) — *(phase R4, with R4.3)* **Keep the constrained types at the C++ boundary** (#2409).

<a id="r4-13"></a>
**R4.13** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-13) — *(phase R4, after it)* **Remove `CFOptional`** (#2417).

<a id="r4-14"></a>
**R4.14** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-14) — *(phase R4, with a large file move - R4.3 or later)* **Group `src/Autogenerated/` by item type.** The directory holds several hundred generated headers side by side.

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
**R4.22** **DONE 2026-09-15 (R4.22.1-R4.22.3)** → [log](exudynRevisionLog2026.md#r4-22) — *(phase R4, after R4.6)* **Star-import surface of the utility modules** (#2438, maintainer request 2026-09-15).

<a id="r4-23"></a>
**R4.23** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-23) — *(phase R4, with the item emitters)* **Generated `itemInterface.py` docstrings do not match the signatures** (#2440).

<a id="r4-24"></a>
**R4.24** **DONE 2026-09-15** — development environments from `[dependency-groups]` in `pyproject.toml`; `docs/requirements.txt` removed (#2441, maintainer request). → [log](exudynRevisionLog2026.md#r4-24)

<a id="r4-25"></a>
**R4.25** **DONE 2026-09-15** — structure members are in the Python interface by default; `SFPybind` on 845 of 883 members replaced by `SFNoPybind` on the other 38 (#2445, maintainer request). → [log](exudynRevisionLog2026.md#r4-25)

<a id="r4-26"></a>
**R4.26** **DONE 2026-09-15** → [log](exudynRevisionLog2026.md#r4-26) — *(phase R4)* **Super element `Vshow` is False when left out of the dict** (#2447).

## R5 — Testing (~3 weeks)  <!-- old Phase 4 -->

<a id="r5-1"></a>
**R5.1** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-1) — **`python/TestModels/test_testModels.py`**: one pytest case per model and mini example, sharing the reference values and tolerances with `runTestSuite.py`, which stays the gate runner.

<a id="r5-2"></a>
**R5.2** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-2) — **Fast vs slow as data**: `SlowTests()` and `OptionalPackageTests()` in `runTestSuiteRefSol.py` drive both `runTestSuite.py --fast` and the pytest markers; nightly stays the full set.

<a id="r5-3"></a>
**R5.3** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-3) — **The last C++ unit tests
    can run again**: a `performUnitTests` build switch (off by default), `PERFORM_UNIT_TESTS` in the
    VS `Debug` configuration, and the two defects that would have skipped or crashed the suite's
    report (#2458).

<a id="r5-4"></a>
**R5.4** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-4) — **The AVX classes have unit tests**

<a id="r5-4-1"></a>
**R5.4.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-1) — *(sub-step of R5.4)* **The matrix variants and the rigid-body/geometry group have unit tests** (#2472).

<a id="r5-4-2"></a>
**R5.4.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-2) — *(sub-step of R5.4)* **Symbolic has unit tests** (#2479).

<a id="r5-4-3"></a>
**R5.4.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-3) — *(sub-step of R5.4)* **`LinearSolver.h` has unit tests** (#2479).

<a id="r5-4-4"></a>
**R5.4.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-4) — *(sub-step of R5.4, from R5.4.1)* **`LinkedDataMatrix(const MatrixBase&)` did not compile** (#2473).

<a id="r5-4-5"></a>
**R5.4.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-5) — *(sub-step of R5.4, from R5.4.1)* **`MatrixContainer::MultMatrixVector` had two preconditions** (#2474).

<a id="r5-4-6"></a>
**R5.4.6** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-6) — *(sub-step of R5.4, from R5.4.5)* **`SparseTripletMatrix(rows, columns, triplets)` kept its size arguments** (#2476).

<a id="r5-4-7"></a>
**R5.4.7** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-7) — *(sub-step of R5.4, from R5.4.2)* **The symbolic headers include what they use** (#2480).

<a id="r5-4-8"></a>
**R5.4.8** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-8) — *(sub-step of R5.4, from R5.4.2)* **A failed symbolic operation frees its nodes** (#2481).

<a id="r5-4-9"></a>
**R5.4.9** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-9) — *(sub-step of R5.4, from R5.4.3)* **The sparse factorization stopped inventing a causing row** (#2482).

<a id="r5-4-10"></a>
**R5.4.10** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-10) — *(sub-step of R5.4, from R5.4.3)* **`LinearSolverType.EigenDense` says what it does not detect** (#2483).

<a id="r5-4-11"></a>
**R5.4.11** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-11) — *(sub-step of R5.4; maintainer request 2026-09-17)* **`pythonTests.cpp` removed** (#2484).

<a id="r5-4-12"></a>
**R5.4.12** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-4-12) — *(sub-step of R5.4; maintainer request 2026-09-17)* **The C++ usage demo of the symbolic types became a readable header** (#2485).

<a id="r5-5"></a>
**R5.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5) — *(phase R5, tooling; decisions D1-D5 answered by the maintainer on 2026-09-17)* **A linter and a type check for the Python side.**

<a id="r5-5-1"></a>
**R5.5.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-1) — *(sub-step of R5.5)* **The generated `__init__.pyi` parses again, and the generator now checks** (#2486).

<a id="r5-5-2"></a>
**R5.5.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-2) — *(sub-step of R5.5)* **The stubs describe the module-level functions and the settings dictionaries** (#2490).

<a id="r5-5-3"></a>
**R5.5.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-3) — *(sub-step of R5.5, half A; decisions D1-D3)* **ruff runs over `python/exudyn/`** (#2487).

<a id="r5-5-4"></a>
**R5.5.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-4) — *(sub-step of R5.5, half B; decisions D4-D5, and the `py.typed` question answered with option (a))* **`stubtest` compares the stubs against the module**: `tools.

<a id="r5-5-5"></a>
**R5.5.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-5) — *(sub-step of R5.5; found by the linter of R5.5.3)* **Four undefined names that raise `NameError` when their code path is reached** (#2488).

<a id="r5-5-6"></a>
**R5.5.6** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-5-6) — *(sub-step of R5.5)*
    **`robotics/future.py` imports `graphics`** (#2489), and its `MakeCorkeRobot` raises instead of
    returning an undefined name - the same pattern as R5.5.5, one function further.

<a id="r5-5-7"></a>
**R5.5.7** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-5-7) — *(sub-step of R5.5)* **The stub gate must pass whether or not the fast module was built** (#2515).

<a id="r5-6"></a>
**R5.6** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-6) — *(phase R5)* **An ASan/UBSan Linux job.** For a C++ library invoking arbitrary user callbacks this catches the class of bug users report as "it crashed with no message".

<a id="r5-6-1"></a>
**R5.6.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-6-1) — *(sub-step of R5.6)* **The sanitizer job is a gate.** It was introduced with `allow_failure.

<a id="r5-7"></a>
**R5.7** **DONE** — rename `pytest.py` - done differently in step R3.1 (`python/pytestTemplate.py`). → [log](exudynRevisionLog2026.md#r5-7)

<a id="r5-8"></a>
**R5.8** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-8) — *(phase R5)* **`runTestSuite.py --parallel[=N]`**: every model in its own interpreter, 22 s → 9-11 s; serial stays the default for the commit gate.

<a id="r5-9"></a>
**R5.9** **DONE 2026-09-11** — Complete and verify the test list. → [log](exudynRevisionLog2026.md#r5-9)

<a id="r5-9-1"></a>
**R5.9.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-9-1) — *(sub-step of R5.9)* **`symbolicModuleTest` fails with numpy 2.2** (#2501).

<a id="r5-9-2"></a>
**R5.9.2** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-9-2) — *(sub-step of R5.9)* **A reference value depended on the numpy version** (#2502).

<a id="r5-9-3"></a>
**R5.9.3** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-9-3) — *(sub-step of R5.9; the other half of R5.9.1)* **`symbolicModuleTest` compares scalars with exact equality** (#2509).

<a id="r5-10"></a>
**R5.10** **DONE 2026-09-10** — `testRunnerTools.ResolveLogFile()` decides the log target before the first write. → [log](exudynRevisionLog2026.md#r5-10)

<a id="r5-11"></a>
**R5.11** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-11) — *(phase R5, release testing)* **Every compiled variant is covered by the release tests** (#2495).

<a id="r5-11-1"></a>
**R5.11.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-11-1) — *(sub-step of R5.11; maintainer decision 2026-09-16)* **How much of the matrix the fast variant needs.**

<a id="r5-11-2"></a>
**R5.11.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-11-2) — *(sub-step of R5.11; found by a maintainer question about the version string)* **"Is this the fast module" is not "does it have AVX2"** (#2496).

<a id="r5-12"></a>
**R5.12** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-12) — *(phase R5, small)* **Test and example hygiene** (#2368, #2377).

<a id="r5-12-1"></a>
**R5.12.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-12-1) — *(sub-step of R5.12)* **`CompositionRuleForRotationVectors` returns 2π instead of 0** (#2494).

<a id="r5-13"></a>
**R5.13** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-13) — *(phase R5, with R5.8 and R5.9)* **Test-suite output goes to its own directory** (#2418, #2454): `exudyn.config.outputDirectory` and one output directory per model; no model writes next to itself any more.

<a id="r5-13-1"></a>
**R5.13.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-1) — *(sub-step of R5.13)* **Stop writing what nothing reads** (#2492).

<a id="r5-13-2"></a>
**R5.13.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-2) — *(sub-step of R5.13)* **Five examples wrote sensor output next to themselves** (#2475).

<a id="r5-13-3"></a>
**R5.13.3** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-3) — *(sub-step of R5.13, split out of R5.13.1)* **Generated FEM data leaves the tracked input directory** (#2491).

<a id="r5-13-4"></a>
**R5.13.4** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-4) — *(sub-step of R5.13; maintainer request 2026-09-17)* **Every writer creates its own output directory** (#2493).

<a id="r5-13-5"></a>
**R5.13.5** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-13-5) — *(sub-step of R5.13; found while answering a maintainer question)* **The suite log is one file again when `EXUDYN_OUTPUTDIRECTORY` is set** (#2500).

<a id="r5-14"></a>
**R5.14** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-14) — **Dev tools are declared**:
    a `test` dependency group (`pytest`, `pytest-xdist`), the `build` group matched to
    `build-system.requires` and to the cibuildwheel version CI pins.

<a id="r5-14-1"></a>
**R5.14.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-14-1) — *(sub-step of R5.14; approved by the maintainer 2026-09-18)* **One set of action versions** (#2463).

<a id="r5-15"></a>
**R5.15** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-15) — **Performance suite reports single runs**

<a id="r5-16"></a>
**R5.16** **DONE 2026-09-16** → [log](exudynRevisionLog2026.md#r5-16) — **Examples run in parallel**
    with a short timeout: each example in its own interpreter and its own output directory, a timeout
    after the solver was reached counts as a pass. 360 s → 49 s.

<a id="r5-17"></a>
**R5.17** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-17) — *(phase R5, after R5.16)* **A switch that stops Exudyn opening windows**, so that a model or an example run outside the test suite does not pop up the renderer - and so that the runners can stop rewriting the source to prevent it.

<a id="r5-17-1"></a>
**R5.17.1** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r5-17-1) — *(sub-step of R5.17)* **A script that draws its own plots ignored the flag** (#2478).

<a id="r5-18"></a>
**R5.18** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18) — *(phase R5, last step; maintainer request 2026-09-18)* **One driver instead of sixteen batch files** (#2503).

<a id="r5-18-1"></a>
**R5.18.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-1) — *(sub-step of R5.18)* **The two runners without an exit code** (#2504).

<a id="r5-18-2"></a>
**R5.18.2** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-2) — *(sub-step of R5.18)* **The examples decide by what the environment HAS** (#2507).

<a id="r5-18-4"></a>
**R5.18.4** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-4) — *(sub-step of R5.18; maintainer supplied the files)* **The missing data files, restored and pruned** (#2510, #2511).

<a id="r5-18-6"></a>
**R5.18.6** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-6) — *(sub-step of R5.18; maintainer request 2026-09-18)* **`exudev build --env NAME`** (#2518): the environment is asked which Python it has, instead of the driver refusing to guess from its name.

<a id="r5-18-5"></a>
**R5.18.5** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r5-18-5) — *(sub-step of R5.18)*
    **The stub check refuses to run against a stale wheel** (#2517), and `exudev build --env`
    installs into the generator environment. Both halves the step offered, and the refusal is
    the one that holds outside the driver.


<a id="r5-18-3"></a>
**R5.18.3** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r5-18-3) — *(sub-step of R5.18; found by the first GitLab run after the driver landed)* **A gate that was green locally and red in CI** (#2508).

<a id="r5-18-7"></a>
**R5.18.7** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r5-18-8) *(sub-step of R5.18)*
    **A gate that fails at random is worse than no
    gate** (#2551). `checkPython.py --stubs --check` reports
    *"exudyn.misc.resultsMonitor._ControlPanel.tk is not present at runtime"* on roughly one run in
    three, with nothing changed in between (measured: one failure in three consecutive runs).
    `_ControlPanel` derives from a tkinter widget, and `tk` exists only once a `Tk` instance has
    been created, so whether stubtest sees it depends on import order or on a display. Either make
    the probe deterministic or put the name in the curated noise list **with the reason** — an
    intermittent gate trains everyone to rerun until green, which costs more than it protects.

    Found while working on R7.1.5 and **not caused by it**.

    *(Numbered R5.18.6 when it was written and corrected to R5.18.7 on 2026-09-20: R5.18.6 was
    already taken by the `exudev build --env NAME` step of 2026-09-18. Step numbers are
    permanent, so the one that was never used is the one that moves.)*

<a id="r5-18-8"></a>
**R5.18.8** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r5-18-8) *(sub-step of R5.18)*
    **A module deleted from the package
    is still shipped in the wheel** (#2560). After `resultsMonitor.py` moved to
    `exudyn/misc/`, the installed package still held the old file: `exudev build` installs with
    `pip --force-reinstall --no-deps`, and the stale module and its `.pyc` survived. stubtest then
    walks the *installed* package, finds `exudyn.resultsMonitor`, cannot import it and reports an
    error about a module that no longer exists.

    **Measured 2026-09-20, and it is worse than an install artefact.** The stale copy lives in
    **`build/lib.<platform>/exudyn/`**, the tree setuptools copies the package from: all five
    modules that moved in R11.4.1 were still there, and
    `dist/exudyn-1.11.212.dev1-cp313-cp313-win_amd64.whl` **shipped `exudyn/resultsMonitor.py`**,
    a file that does not exist in the source any more. A release built without cleaning would ship
    deleted modules to users.

    It also **masked a real bug**: `exudyn/__init__.py` still did
    `from .mainSystemExtensions import ...`, which only worked because the stale copy was there.
    Deleting the `build/lib.*/exudyn` trees made the import fail immediately, which is how it was
    found.

    So: `exudev build` removes the package tree under `build/lib.*` before building (or passes a
    clean build directory), and the release step verifies that the wheel's module set matches the
    source. It cost time twice in one day: here, and through a leftover `__main__`.

## R6 — Error handling and UX (ongoing, after R2)  <!-- old Phase 5 -->

An error has four separable properties, and mixing them is what made this phase read as a list of
unrelated items. Each step below fixes exactly one of them:

| property | what it means | steps |
|---|---|---|
| **which exception type** reaches Python | after R6.7: `TypeError` and `ValueError` for parameters, `RuntimeError` for everything else. R6.3 adds `IndexError`, the arithmetic types, and Exudyn's own classes for a model error, a solver failure and an internal defect | **R6.7** **DONE** parameter errors · **R6.3** everything else · **R6.2** the import itself |
| **what the message says** | the text, and where the traceback points | **R6.4** the taxonomy R6.3 and R6.7 write against |
| **where the message is written** | console, log file, or lost | **R6.8** — `CHECKandTHROW` writes to no file at all |
| **what it costs to raise** | errors in a loop are a normal pattern, not an exceptional one | **R6.6** |
| **whether the check runs at all** | opting out for a production run | **R6.5** **DONE** |
| how the **Python side** handles errors | `python/exudyn/` catching its own | **R6.1** |

**Recommended order:** **R6.7** **DONE** → **R6.3** (the same treatment for everything else, and the
message that R6.7 deliberately left alone) → **R6.8** (where that message is written) → **R6.4**
(write down what R6.3 established) → **R6.1**. **R6.2** and **R6.6** touch none of the above and can be
done at any time; **R6.2 is a hard prerequisite for phase R9**.

Measured 2026-09-18: `PyError` threw `std::runtime_error` for every user error (`Stdoutput.cpp:333`)
until R6.7 gave it a type — **597** call sites, 20 of them in `PyConversion.h`.
`CHECKandTHROW` has **818** call sites, and `python/exudyn/` has **46** bare `except:` clauses, the largest groups in `processing.py`, `interactive.py`, `solver.py` and
`__init__.py` (7/7/6/6). The behaviour is recorded probe by probe in
`parameterConversionTest.py`, so a change of exception type shows up as a reviewable diff of
`parameterConversionTestReference.txt` rather than as a surprise.


<a id="r5-18-10"></a>
**R5.18.10** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r5-18-10) *(sub-step of
    R5.18)* **The regenerate step of `--all-checks` could not report tier 1 drift** (#2563). `tools/regenerate.py`
    fails on tier 1 drift only with `--check`, and `exudev generate --all-checks` runs it without,
    because its job there is to regenerate. The drift is *printed* and the step is reported **ok**.

    That is how R11.4.5 changed `python/exudyn/types/items.py` — a tier 1 file — without anything
    saying so: `typesEmitter.py` looked for the parent header in `src/Objects`, the folder had been
    renamed, and two items lost their type bits. Caught by reading an uncommitted diff.

    Either run the comparison again in check mode after regenerating, or let that step's summary
    line say **TIER 1 DRIFT** instead of *ok*.

<a id="r5-18-9"></a>
**R5.18.9** *(sub-step of R5.18, added 2026-09-20)* **Nothing in the test suite ever calls
    `UpdateGraphics`** (#2562). The drawing code of every item — ~4,000
    lines — runs only when the renderer runs, and every runner sets
    `EXUDYN_SUPPRESS_UI_WINDOW_OPEN`. Moving all 79 of those functions in R11.4.4 could therefore be
    verified only by the compiler and by comparing the text of the bodies before and after.

    What would make it testable: a headless path that updates the graphics data of a model and
    returns a summary of it (triangles, lines, texts per item). The data already exists in
    `VisualizationSystemData`; only the binding and the comparison are missing.


**R6.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-1) — *(phase R6)*
    **Every bare `except:` in `python/exudyn/` names what it catches** (#2539): 52 of them in 16
    files, plus a docstring example. The 16 `E722` entries of the ruff baseline are gone, and the
    gate is the regression test.


<a id="r6-2"></a>
**R6.2** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r6-2) — *(phase R6)*
    **The binary selection is one testable function** (#2540) that returns its log, prints it
    under `EXUDYN_IMPORT_VERBOSE`, and raises a single `ImportError` listing every candidate and
    why it was skipped or failed. Prerequisite for phase R9, done.


<a id="r6-3"></a>
**R6.3** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r6-3-done) — *(phase R6, after R6.7)* **Give every error
    the type it deserves, and the helpers the shape they need.** The maintainer released the four
    helpers for revision in this step: `PyError`, `SysError`, `PyWarning` and `CHECKandTHROW` may
    be rewritten, not only re-typed.

    **The evidence is fact 30 of the info document**: 2249 call sites — `CHECKandTHROW` 816,
    `CHECKandTHROWstring` 378, `PyError` 587, `PyWarning` 262, `SysError` 206. Three findings
    decide the shape of the step, and none of them is the type mapping:

    1. **The largest single category is not an error.** 199 of the sites are deprecation notices,
       184 of them generated into `Autogenerated/VisualizationSettings.h`. They belong in Python's
       warning machinery (`DeprecationWarning`), where a user can filter or promote them — not in
       an error helper. Splitting them out removes a tenth of the inventory from the question.
    2. **The two largest areas are not user-facing.** Linalg (486, almost all `CHECKandTHROW` in
       `Matrix.h`, `ConstSizeMatrix.h`, `ConstSizeVector.h`) and Autogenerated (462) fire on an
       **Exudyn bug**, not a user mistake. Giving those a friendly Python type would dress up a
       defect as a usage question. They keep one internal type. **Triage user-facing against
       internal first**; map only the user-facing remainder.
    3. **`SysError` and `CHECKandTHROW` are nearly the same thing** (maintainer): `CHECKandTHROW`
       exists because it makes a small test cheap to write and cheap to leave in. That is worth
       keeping — so the revision gives `CHECKandTHROW` the missing half (a type, and the file
       output of step R6.8) rather than replacing it with something heavier.

    **The kinds, and what each becomes** (the names in the code are historical and say nothing
    about the kind, because exception types were not a concern when the checks were written):

    | kind | sites | becomes |
    |---|---|---|
    | internal invariant | Linalg, Autogenerated, "invalid call", "untested" | one internal type; the message says "please report" |
    | index | 259 | `IndexError` |
    | size / shape | 249 | `ValueError` |
    | arithmetic | 63 | `ZeroDivisionError` / `ArithmeticError` |
    | illegal operation | 612 | the Exudyn user error, see below |
    | solver failure | 19 named, 35 `PyError`/`SysError` in `src/Solver/` | its own type |
    | deprecation | 199 | `DeprecationWarning`, not an error at all |

    **pybind11 supports all of this, and it was checked rather than assumed** (vendored 2.12.1,
    build environment 2.13.6): `py::index_error`, `py::key_error`, `py::value_error`,
    `py::type_error`, `py::attribute_error`, `py::buffer_error`, `py::import_error` are builtin and
    need nothing but a `throw`. There is **no** builtin for `ArithmeticError`, `ZeroDivisionError`
    or `OSError`: those go through `py::set_error(PyExc_ZeroDivisionError, message)` followed by
    `throw py::error_already_set()`. Custom classes come from
    `py::register_exception<T>(scope, "Name", base)`.

    **Decisions (maintainer, 2026-09-18).**

    1. **Exudyn defines its own exception classes, and everything it raises from C++ is one of
       them** ("wrapping everything as ExudynError sounds good"). Built-in types cannot express
       "the solver diverged" or "this combination of settings is illegal", and a user who needs to
       tell them apart is left matching on message text. Each class derives *additionally* from the
       built-in that fits, so `except exudyn.ExudynError` catches everything Exudyn raises while
       the built-in stays available. **This is not free of consequence, and the honest form of it
       is**: a site that becomes `SolverError` or `InternalError` keeps being caught by an existing
       `except RuntimeError`; a site that becomes `ModelError`, `ExudynIndexError` or
       `ExudynValueError` is **not** a `RuntimeError` any more, and an `except RuntimeError` around
       it stops catching. That is the intended change - the old type was wrong - but it is a
       user-visible one, so R6.3.6 maps area by area and names what changes in each.
    2. **Scope: the C++ side.** The Python utility modules (`FEM`, `robotics`, ...) keep raising
       ordinary built-ins, the way numpy does. This taxonomy is for what crosses the C++/Python
       boundary, which is where the message is otherwise the only clue.
    3. **Nothing becomes fatal.** `CHECKandTHROW` today raises an exception a parameter variation
       *can* catch, if the user writes the `try/except` - and it must stay that way, "because
       otherwise the user may report some 0 or None values and does not see where it happens".
       Internal errors are catchable like every other; the solver simply does not catch them.
    4. **"Not implemented" is its own kind**, between user error and internal error. A feature
       combination that does not exist is not a mistake and not a bug. It gets its own class, and
       with it a documentation rule: **the docs do not enumerate which combinations work** - lists
       like that go stale - the code says so when asked, and that matters most for experimental
       parts.
    5. **Internal errors stay diagnosable.** A developer reading a long, non-reproducible run out of
       a user's feedback needs to know *what* failed, not only that something did: the internal type
       keeps the check text and the source location. The type says "report this"; the message says
       what to report.
    6. **Deprecations become a real `DeprecationWarning`**, through one dedicated C++ function, so
       that the 199 sites are unified and findable.

    ```
    exudyn.ExudynError(Exception)                           the root; catch this to catch everything
      exudyn.ModelError(ExudynError, ValueError)            illegal combination, wrong setting
      exudyn.SolverError(ExudynError, RuntimeError)         singular Jacobian, no convergence, divergence
      exudyn.InternalError(ExudynError, RuntimeError)       an Exudyn bug; please report it
      exudyn.NotImplementedFeatureError(ExudynError, NotImplementedError)
      exudyn.ExudynIndexError(ExudynError, IndexError)      the built-in mirrors keep the Exudyn
      exudyn.ExudynValueError(ExudynError, ValueError)      prefix on purpose: 'from exudyn import *'
      exudyn.ExudynTypeError(ExudynError, TypeError)        must not shadow a built-in name
      exudyn.ExudynArithmeticError(ExudynError, ArithmeticError)
    ```

    The mechanism this rests on: pybind11's `exception` constructor passes `base` straight to
    CPython's `PyErr_NewException` (`pybind11.h:2616`), and that accepts **a class or a tuple of
    classes** - so two bases need nothing but `py::make_tuple(...)` as the `base` handle. Confirmed
    by building and running it, sub-step R6.3.1; the fallback, had it not held, was single
    inheritance from the built-in plus an `ExudynError` marker.

    **Also the message.** `PyError` prints the detail to `pout` and then throws a fixed string,
    `"Exudyn: parsing of Python file terminated due to Python (user) error"` (`Stdoutput.cpp:333`),
    so `str(exception)` never carries what actually went wrong. R6.7 deliberately left that alone
    to keep one change in one step; it belongs here.

    Chain `py::error_already_set` as well, so user-function tracebacks survive instead of being
    stringified (`ExceptionsTemplates.h:50`).

    **The trap to respect throughout** (info fact 29): `EXUexception` is a `#define` for
    `std::runtime_error` and every pybind11 exception derives from it, so a
    `catch (const py::builtin_exception&) { throw; }` must come **before** any
    `catch (const EXUexception&)` in the same try block. The new classes derive from
    `EXUexception` as well, for exactly the same reason and with exactly the same consequence.

    **Sub-steps.** 2249 call sites cannot be one commit; each of these is a gate-passing change on
    its own, and the order is the order of dependency.

<a id="r6-3-1"></a>
**R6.3.1** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-1) — *(sub-step of R6.3)*
    **The nine exception classes exist, are exported, and the mechanism holds** (#2516) — including
    the third instance of the flattening trap, which the one converted call site caught at once.

<a id="r6-3-2"></a>
**R6.3.2** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-2) — *(sub-step of R6.3)*
    **Triage: who is each of the 2064 messages for?** (#2520) `tools/errorTriage.py` sorts them into
    1082 user-facing, 793 internal and 189 that have to be read; one of the guesses the step started
    from was wrong.

<a id="r6-3-3"></a>
**R6.3.3** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-3) — *(sub-step of R6.3)*
    **The helpers learn to carry a type** (#2521). An optional last argument on `CHECKandTHROW` and
    `CHECKandTHROWstring`, nine kinds on `PyErrorType`, and one `SysError` default change that
    the triage of R6.3.2 had already predicted would hit six sites.

<a id="r6-3-4"></a>
**R6.3.4** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-4) — *(sub-step of R6.3)*
    **Deprecations leave the error path** (#2522). `PyDeprecated` raises a real
    `DeprecationWarning`; 199 sites converted; a deprecated setting read 500 times reports once,
    and reports the user's own file and line.

<a id="r6-3-5"></a>
**R6.3.5** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-5) — *(sub-step of R6.3)*
    **The message survives** (#2527), the location names the **user's** Python frame and not
    `solver.py`, and a user-function error is no longer reported a second time as an internal
    Exudyn error (#2524).

<a id="r6-3-6"></a>
**R6.3.6** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-6-done) — *(sub-step of R6.3)*
    **The mapping** (#2528): 1082 user-facing error sites read area by area in seven commits, and
    the untyped `CHECKandTHROW` default turned from `EXUexception` into `ExudynInternalError`.
    Nothing in Exudyn raises a bare `RuntimeError` any more except the ten `Add*` wrappers, which
    do not know the type they caught.

<a id="r6-3-7"></a>
**R6.3.7** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-7) — *(sub-step of R6.3;
    maintainer request 2026-09-18)* **A model that shows what an Exudyn error looks like**
    (#2523): ten provoked user errors including one inside a Python user function, each caught and
    reported with its class and message - and a switch that lets one of them fly, to see it the way
    Spyder or VS Code shows it. It found #2524 on the first run.


<a id="r6-3-8"></a>
**R6.3.8** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-8) — *(sub-step of R6.3)*
    **The original Python exception is chained as `__cause__`** (#2537), with its traceback,
    instead of being stringified into the message.

<a id="r6-3-9"></a>
**R6.3.9** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-9) — *(sub-step of R6.3; added on the
    maintainer's request, 2026-09-18)*
    **The rules get a home** (#2529). The nine classes, the seven helpers and the rules for
    choosing between them existed only in code comments and in the revision log — a record,
    not a reference. They are now `docs/dev/CODING_STYLE.md` §10, with `CONTRIBUTING.md`,
    `docs/dev/README.md` and `CLAUDE.md` pointing at that one place.

<a id="r6-3-10"></a>
**R6.3.10** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-10) — *(sub-step of R6.3; added on the
    maintainer's decision, 2026-09-18)*
    **The error block goes to the log file and never to the console** (#2530). The exception
    already carries the same message and the same location, and a *caught* exception must not
    flood the terminal. Both file channels now write the identical text.

<a id="r6-3-11"></a>
**R6.3.11** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-11) — *(sub-step of R6.3)*
    **A typed exception from the solver stops the renderer again** (#2531): the flag is raised
    at the solver boundary, where the maintainer placed it, and `StopRendererOnError()` is now
    the one place that knows the rule.

<a id="r6-3-12"></a>
**R6.3.12** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-12) — *(sub-step of R6.3)*
    **Eight user-facing "not implemented" sites stop calling themselves SYSTEM ERROR** (#2532):
    `SysError` → `PyError`, keeping the type R6.3.6 gave them.

<a id="r6-3-13"></a>
**R6.3.13** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-3-13) — *(sub-step of R6.3; maintainer
    decision 2026-09-18)*
    **`SolveStatic` and `SolveDynamic` raise what actually failed** (#2534), the overridden
    `simulationSettings` are restored even when a solve fails (#2535), and the explicit dynamic
    solver stops failing silently (#2536).

<a id="r6-4"></a>
**R6.4** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r6-4) — *(phase R6)*
    **The error taxonomy is in the user documentation** (#2542): what each of the nine types
    means, what to do about it, and how to catch a solver failure to retry or to score a failed
    run. `introduction.tex`, section *Errors: what Exudyn raises, and what to do about it*.

    **Phase R6 is complete with this step.**


<a id="r6-5"></a>
**R6.5** **DONE 2026-09-15** — A user switch for parameter range checks. → [log](exudynRevisionLog2026.md#r6-5)

<a id="r6-6"></a>
**R6.6** **DONE 2026-09-19 — absorbed by R6.3.5** → [log](exudynRevisionLog2026.md#r6-6) — *(phase R6)*
    `inspect.getframeinfo` is gone, `inspect.currentframe()` is kept, and a user function still
    reports its own line — which was the acceptance criterion. The 9 s of #2423 did not
    reproduce; the measurement is in the log.


<a id="r6-7"></a>
**R6.7** **DONE 2026-09-18** — a wrong parameter raises `TypeError` when the object cannot be that
    parameter at all and `ValueError` when the kind is right and the value is not; `PyError` chooses
    the Python exception, and the three layers that used to flatten it back to `RuntimeError` were
    found and fixed (#2432). → [log](exudynRevisionLog2026.md#r6-7)

<a id="r6-8"></a>
**R6.8** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r6-8) — *(phase R6)*
    **An error reaches the log file, whichever helper raised it** (#2538). One writer:
    `CSolverBase::SolveSystem` catches what ends a run where the solver file is known. The
    `std::ofstream&` overloads of `PyError` and `SysError` are gone with it, and the rule for all
    three channels is in `CODING_STYLE.md` §10.7.


<a id="r6-8-1"></a>
**R6.8.1** **DONE 2026-09-20** *(sub-step of R6.8)* **A test that passed by luck** (#2561). The
    solver-file tests of R6.8 call `mbs.SolveDynamic`, which exists only once
    `exudyn.utilities` (or `exudyn.misc.mainSystemExtensions`) has been imported — and
    `test_exceptions.py` imported neither. Under `pytest -n 8` it passed as long as the worker
    that ran it had already run a test file that does import them. Adding one test model
    changed the distribution, and three tests failed with `FileNotFoundError`, because the
    `AttributeError` was swallowed by the `except BaseException` of the helper.

    Fixed by importing `exudyn.utilities` in the test file. **What stays open** and is worth a
    sweep when R5 is revisited: a test file that depends on an import made in another test file
    passes by luck, and a helper that catches `BaseException` and reports only its own next
    failure hides which error actually happened.

<a id="r6-9"></a>
**R6.9** **DONE 2026-09-19** *(maintainer session; integrated 2026-09-20)* **The results monitor
    becomes `exudyn.misc.resultsMonitor`** (#2557). The 2021 script inside the package — it read
    `sys.argv` and called `plt.ion()` while being *imported* — is a module now:
    `MonitorResults(...)` callable from a script or Spyder, an `argparse` CLI behind it, file
    selection by dialog or `--last` instead of a file name that had to be typed in the right
    directory, a tkinter control panel (stdlib, no new dependency), a settings file in
    `~/.exudyn/`, incremental reading instead of re-parsing the whole file on every tick, and
    `--once`, which is what makes it testable at all: `python/TestModels/resultsMonitorTest.py`
    (1.2 s) covers the four file types, the incremental reader including a torn last line, the
    buffer reset after a file is overwritten, and eight command-line return codes.

    It also removes three pieces of debt: the ruff `E402` baseline entry, the stubtest baseline
    entry and the `allExudynModulesTest` exclusion — the module is importable now, so the test
    covers it.

    Written in a parallel session; this session integrated it, which meant the reference-solution
    entry, the `excludeModules` list (where `__main__.py` had to take the place the monitor left,
    because importing a `__main__` *runs* it), the two `processing.py` docstrings and the example
    that tells the reader how to watch its own output.

## R7 — Documentation (~3 weeks)  <!-- old Phase 6 -->

<a id="r7-1"></a>
**R7.1** **DONE 2026-09-21** — all nine sub-steps (R7.1.1 to R7.1.9) are closed.
    Sphinx (readthedocs) stays; the sources became **MyST Markdown** (`myst-parser`, dev-only):
    hand-written chapters converted from `.tex`, generated reference pages from the docs
    emitters as `.md`, remaining `.rst` files converted as they were touched; new documentation
    is Markdown from now on (decision 2026-09-15). ~~The PDF is generated via `latexpdf` (front page
    through `latex_elements`).~~ — superseded by decision **D8**: there is no PDF in 2.0.
    `latexConverter.py` and `doc2rst.py` are deleted (R7.1.7).

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

    **ANSWERED (maintainer, 2026-09-19; info document D8): `theDoc.pdf` does not survive 2.0.**
    The LaTeX documentation build ends with this phase; `.tex` stays possible as a source only in
    special, tiny cases, and none is foreseen. So R7.1.6 carries **no `latexpdf` path**, the front
    page and the PDF table of contents are not carried over, and #2545 - the remaining LaTeX
    escaping errors of the emitters - is closed by deletion rather than fixed.

    After latex has been abandoned, as well as the doc2rst.py (check if there is something that 
    will be still needed from the old latex converters, like the abbreviation list at the end of
    doc2rst, etc.). Further, the autoGenerateHelper.py - which is in a terrible state, probably 
    most terrible in the project - will not require most of its functions, so cleanup is needed.

<a id="r7-1-1"></a>
**R7.1.1** **DECIDED 2026-09-19: mermaid** *(sub-step of R7.1; maintainer question 2026-09-17,
    answered 2026-09-19; info document D9)* **What happens to the tikz figures.** The recommendation
    below was taken: the 13 tikz pictures become mermaid diagrams, and each hand-made PNG twin is
    deleted with the tikz source it duplicated. With D8 there is no PDF, so the pre-rendering to SVG
    that the recommendation mentions is not needed either - mermaid text is the single source, and
    MyST renders it. This unblocks the three chapters of R7.1.5 that contain figures.
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
**R7.1.2** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r7-1-2) — *(sub-step of R7.1;
    done out of order because it was the last red job in CI)* **A generated page was in no
    toctree** (#2505). `docs/RST/TestModels/sphereTriangleTest.rst` produced
    `WARNING: document isn't included in any toctree`, and the GitLab docs job runs
    `sphinx-build -W`, so that single warning failed the job.

    **It was a stale generated file, not a missing index entry.** `TestModelsIndex.rst` is built
    from the keys of `TestExamplesReferenceSolution()`, and `sphereTriangleTest.py` moved into
    `DeliberatelyNotRun()` - its explicit integrator goes unstable, phase R10. The generator
    therefore stopped listing it *and* stopped writing its page; the page from before simply stayed
    behind. Confirmed by deleting it and regenerating: it is not recreated. Removed, and
    `sphinx-build -b html . _build -E -W --keep-going` then succeeds.

    **`exudev docs` is now strict by default** (`-W --keep-going`, with `--no-strict` to opt out),
    because the local build must apply the gate CI applies - a warning that passes here and fails
    there is the failure mode of #2508 in a second tool.

<a id="r7-1-3"></a>
**R7.1.3** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r7-1-3) — *(sub-step of R7.1; the
    repair half of the maintainer's "A then C", 2026-09-19)*
    **`theDoc.pdf` can be built again** (#2543, #2544). Four causes, all restructuring fallout;
    the largest was that `issueTracker.py` wrote issue text into LaTeX without escaping it.
    Errors went 158 → **74**, and a complete PDF is produced where none was before. The 74
    that remain are the same class in other emitters and are #2545 — worth fixing only if the
    PDF survives R7.1.

<a id="r7-1-4"></a>
**R7.1.4** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r7-1-4) — *(sub-step of R7.1; step 1 of
    the maintainer's "A then C", 2026-09-19)*
    **The Markdown that already exists becomes documentation** (#2546). `myst-parser` in the
    docs dependency group and in `conf.py`, and eleven Markdown files in the toctree: the four
    `docs/dev/` documents, `CONTRIBUTING.md`, the READMEs of `definitions/`, the generators and
    `exudev`, and the three `docs/howTo/` notes that are really Markdown.

    It is also the cheapest possible test of the MyST path the rest of R7.1 rests on — on the
    project's own files rather than on a sample — and it paid immediately: the strict build
    found eleven broken cross-references nobody could see while the files were clone-only, and
    #2547, five how-to files that are `.txt` renamed to `.md`.

<a id="r7-1-8"></a>
**R7.1.8** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r7-1-8) *(sub-step of R7.1;
    decided before R7.1.5 started. Numbered 8 because step numbers are
    permanent and R7.1.5-R7.1.7 were written first; it runs before them.)*
    **One directory is either generated or hand-written, and says which** (#2548). Raised by the maintainer
    2026-09-19 after a hand edit to `index.rst` - a generated file that looks hand-written - was
    silently reverted by the next `tools/regenerate.py` in step R7.1.4.

    **The measured situation.** Nothing in the tree marks a generated file: no banner, no separate
    directory, no `.gitattributes`. Today `docs/RST/` (~498 `.rst`), `index.rst` and `README.rst`
    are entirely generated by `doc2rst.py`; `docs/theDoc/` is **mixed** - 9 hand-written `.tex`
    chapters next to 8 emitter-written `.tex` files, `trackerlog.tex` and `versionName.txt` from the
    issue tracker, and the LaTeX build products (`theDoc.pdf`, `.aux`, `.log`, `.toc`, ...).

    **DECIDED 2026-09-19 (info document D10): the recommendation below is the target layout**, and
    the conversion of R7.1.5 writes into `docs/manual/` from its first chapter on. It is a three-way
    split by *origin*:

    | directory | what | in git |
    |---|---|---|
    | `docs/manual/` | the hand-written chapters, Markdown, the only place a human edits | yes |
    | `docs/generated/` | everything the emitters and the tracker write, Markdown | R7.3 decides; ideally CI-built |
    | `docs/dev/`, `docs/howTo/` | the developer documentation, already Markdown, already human-only | yes |

    plus three cheap guards: a **banner as the first line of every generated file** (`<!-- GENERATED
    by tools/generators/itemDocsEmitter.py - do not edit -->`), a `README.md` in `docs/generated/`
    saying the same, and `docs/generated/** linguist-generated=true` in `.gitattributes` so GitHub
    collapses those diffs. `index.md` becomes **hand-written** and lists the generated index pages
    by path - a table of contents is a human decision, and the generator writing it is precisely
    what went wrong in R7.1.4; the `indexRST` template disappears with `doc2rst.py` in R7.1.7.

    `docs/theDoc/` then holds nothing and is deleted with the LaTeX build (D8), together with
    `docs/RST/`. **Both are tracked-file deletions and need explicit approval** when the step runs.

    **The figures move to `docs/figures/`** (maintainer, 2026-09-19). The 85 images live in
    `docs/theDoc/figures/` today and are referenced from 40 files — the chapters, the generated
    `.rst`, **and four generators** (`doc2rst.py`, `itemDocsEmitter.py`, `itemDefsObjects.py`,
    `itemDefsMarkers.py`), which is why this is **one commit of its own**, made when `docs/theDoc/`
    is emptied rather than piecemeal: a half-moved figure directory breaks both builds at once.
    Until then the converted chapters keep writing `/docs/theDoc/figures/...`, and that one path is
    what the move rewrites.

    **The splitting question of R7.1.5 is decided with it (2026-09-19): convert first, split
    after.** A chapter becomes one `.md` with the same content, which can be diffed against the
    `.rst` that Sphinx renders today; where a chapter is then cut into sub-documents is a separate,
    content-preserving commit, decided with the rendered pages in front of us. A chapter that is cut
    becomes a directory `docs/manual/<chapter>/` with its own index - and the target of at most ~20
    documents out of the 9 chapters holds.

<a id="r7-1-5"></a>
**R7.1.5** **DONE 2026-09-19** (#2549) → [log](exudynRevisionLog2026.md#r7-1-5) *(sub-step of R7.1,
    after R7.1.4)* **Convert the hand-written chapters to Markdown**,
    one commit each. Nine files, ~8,800 lines: `jacobians` (82), `notation` (269), `GUI` (319),
    `theDoc` preamble (513), `gettingStarted` (990), `solver` (951), `tutorial` (1132),
    `introduction` (1588), `theory` (2923) — smallest first, so that the converter questions
    (math, `\refSection`, listings, figures) are answered on a small file before a large one
    depends on the answer.
    Before conversion, suggest and decide with the maintainer how to split the larger files into smaller subfiles (but try to avoid having more than 20 documents out of the 9). Either first only convert, then restructure or do both at the same time - whatever works better.

    **DECIDED 2026-09-19: convert first, split after** (see R7.1.8). Each chapter is converted 1:1
    into one `docs/manual/<chapter>.md`, so the conversion is checkable against the generated `.rst`;
    the cuts are proposed afterwards, per chapter, with the rendered pages available. Target: at
    most ~20 documents out of the 9 chapters.

    **Measured 2026-09-19, before starting: it is 8 files, not 9.** `jacobians.tex` (82 lines) is
    **entirely commented out** - not one line of content is active - and no file `\input`s it:
    neither `theDoc.tex` nor `doc2rst.py` mentions it. It produces no page today and there is
    nothing to convert. Proposal: **delete it** (a tracked file, so it needs approval) rather than
    carry it into `docs/manual/`. The included chapters are `gettingStarted`, `introduction`,
    `tutorial`, `notation`, `theory`, `GUI`, `solver`, plus the `theDoc.tex` preamble.

    **The check is the RST**: each chapter already has a generated `.rst`, so a conversion can be
    compared against what Sphinx renders today rather than judged by eye.

    **#2547 DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r7-1-5-howto): the five
    `docs/howTo/` files that were shell transcripts are real Markdown and are in the documentation.
    Converting them meant rewriting them — `buildFromSource.md` still described Python 3.6, a
    `main/` directory and `bdist_wininst`.

    **ALL SEVEN CHAPTERS ARE CONVERTED (2026-09-19)** with `tools/tex2md.py`, a one-shot converter
    that is deleted again in R7.1.7: `notation`, `GUI`, `solver`, `tutorial`, `introduction`,
    `theory`, `gettingStarted`. `docs/theDoc/` holds no hand-written chapter any more.

    The eighth item of the original list, the `theDoc.tex` preamble (515 lines), is **not
    converted**: it is the LaTeX document skeleton — packages, title page, `\input` list — and
    contributes nothing to the HTML. It dies with the LaTeX build in R7.1.7 (D8).

    `README.rst` became hand-written and a document of the documentation in the same step
    (info document D11).

    **The split is done too (2026-09-19)**: 22 documents, named `<chapterStem><Section>.md` so
    that the chapter is readable from the filename. One commit per chapter, each a pure move
    verified word by word. **R7.1.5 is complete.** Each conversion removes its chapter
    from `filesParsed` in `doc2rst.py` in the same commit, because two copies of a chapter define
    the same labels twice and the strict build stops.

    **Unblocked 2026-09-19**: R7.1.1 decided mermaid, so `introduction`, `solver` and `theory`
    convert their 13 tikz pictures into mermaid blocks and drop the duplicated PNG.

    **After R7.1.8**, which decides where a converted chapter is written to.

<a id="r7-1-9"></a>
**R7.1.9** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r7-1-9) *(sub-step of R7.1)*
    **The tikz twins become mermaid**
    (decision D9, R7.1.1). 13 `tikzpicture` environments in `introduction`, `solver` and `theory`,
    each with a hand-made PNG beside it that the HTML has always shown instead. One pass over all
    three chapters rather than a third of the job in each chapter commit: the diagrams share a
    style, and the point of the step is that the **duplicate ends** — the mermaid text becomes
    the single source and the PNG is deleted.

    Until this step runs, a converted chapter keeps the PNG it shows today, so the documentation
    never loses a figure in between.

<a id="r7-1-6"></a>
**R7.1.6** **DONE 2026-09-20** (#2565, #2545) → [log](exudynRevisionLog2026.md#r7-1-6)
    *(sub-step of R7.1, after R7.1.5)* **The emitters write Markdown.** **Done, all five emitters** —
    `interfaces.tex`, `pythonUtilitiesDescription.tex`, `manual_interfaces.tex`,
    `MainSystemExt.tex`, `MainSystemCreateExt.tex`, `itemDefinition.tex`, `trackerlog.tex` and all
    of their RST pages are replaced by 158 Markdown pages in `docs/generated/`: `structures/` (7),
    `pythonUtilities/` (31), `cInterface/` (10), `items/` (109) and `trackerlog.md`. #2545 is
    resolved with the escaping moved to `issueTracker.ToMarkdown` and tested;
    **#2550 stays open** — the Markdown carries the citation keys in brackets, which is
    readable but not linked. The larger half by
    volume — 8 generated `.tex` files, ~43,000 lines — and the smaller half by risk, because
    it is emitter code and not prose. `itemDocsEmitter`, `structureDocsEmitter`,
    `utilityDocsEmitter`, `mainSystemExtensionDocsEmitter` and the tracker's own writer emit `.md`
    instead of `.tex` + `.rst`.

    **#2545 is resolved here, not abandoned**: the 74 remaining errors are *LaTeX* escaping errors,
    and they exist only because the emitters write LaTeX - deleting the LaTeX writers ends them at
    the source, which is why fixing them separately would be work thrown away. But the requirement
    behind the issue does **not** disappear and moves with the code: **the Markdown writers escape
    too**, at the writer, where the output format is known. Different characters (`*`, `_`, `` ` ``,
    `#`, `|` in tables, `<`), same rule - an author writes text, not markup. #2545 is resolved with
    that note when this step lands, and the escaping test that comes with it is what proves it.

    **No `latexpdf` path** (maintainer, 2026-09-19; info document D8): the PDF does not survive
    2.0, so this step emits Markdown only and the LaTeX writers of the emitters are deleted rather
    than ported. `docs/theDoc/` loses its generated `.tex` files here, which is half of R7.1.8.

<a id="r7-1-7"></a>
**R7.1.7** **DONE 2026-09-21** (#2548) — [log](exudynRevisionLog2026.md#r7-1-7) — *(sub-step of R7.1, last)*
    **Delete the converters**: `src/pythonGenerator/doc2rst.py` (733 lines), `latexConverter.py`
    (836) and the parts of `autoGenerateHelper.py` (1900) that only served them. Check first what
    has to survive — the abbreviation list at the end of `doc2rst.py` is named in R7.1 as one
    such thing.

    **Done in the first commit**: `doc2rst.py`, `latexConverter.py`, `makeAllBinariesScripts.py`,
    `index.rst`, **`docs/RST/` (291 files)** and **`docs/theDoc/`** including `theDoc.pdf` are
    deleted (approved by the maintainer 2026-09-21, PDF kept in the history); what `doc2rst.py`
    did besides converting is the new `examplesDocsEmitter.py` (290 pages: the examples, the test
    models, the abbreviations); `index.md` is hand-written (D10) and `README.rst` stays
    hand-written with its version line stamped by the tracker (D11); the figures moved to
    `docs/figures/` with 172 references in 31 files. `tools/tex2md.py` could **not** be deleted
    — R7.1.6 made it the converter every emitter calls — and became
    `tools/generators/latexToMarkdown.py`.

    **Second commit**: the LaTeX and RST branches inside the emitters and inside `PyLatexRST` are
    deleted — 1,081 lines out of `autoGenerateHelper.py` and `utilityDocsModel.py` alone. The
    generated output was snapshotted first (773 files) and differs afterwards in seven places,
    all additions: a note that the LaTeX and RST branches carried and the Markdown never had.

    **This is also where R7.3 becomes possible**: with no `.tex` → `.rst` conversion, the 498
    committed generated `.rst` files stop being an input to anything and can be built in CI
    instead of committed.

<a id="r7-2"></a>
**R7.2** *(after R7.1 — and, by the maintainer's decision of 2026-09-21, **after the tracker
    changes of R8**, so that they are carried into the documentation in the same pass rather than
    twice)* **Carry the revision into the documentation.**
    Extract every change recorded in this plan and in `exudynRevisionLog2026.md` - new flags and
    switches (e.g. `exudyn.special.exceptions.parameterRangeChecks`), conversion and error behaviour,
    definitions and generators, howto build on each platform (put the simple way also into the main README), tools and workflow - and update the user and developer documentation accordingly. The plan and log are records, not documentation; afterwards the plan is reduced to an archive.

<a id="r7-3"></a>
**R7.3** **DONE 2026-09-21 (decision)** — [log](exudynRevisionLog2026.md#r7-3) — *(phase R7)*
    Stop committing generated RST and `theDoc.pdf`; build in CI, publish the PDF as a release
    asset.

    **Two of the three halves are void when the step is reached.** `theDoc.pdf` is deleted in
    R7.1.7 and the PDF does not survive 2.0 (D8), so there is nothing to publish as a release
    asset; `docs/RST/` is deleted in the same step, so there is no generated RST to stop
    committing. What is left is `docs/generated/`, the 450 Markdown files the emitters and the
    tracker write.

    **Decided 2026-09-21 (info document D12): they stay committed.** Three reasons, in order of
    weight: the reference manual stays readable on GitHub for anyone without the toolchain;
    `regenerate.py`'s tier-2 check compares the generators against the commit and is the only
    thing that would notice an emitter changing its output by accident — it has no meaning for
    files that are not committed; and the documentation build stays free of a generator step,
    where Read the Docs would need one *and* a corrected skip rule, since its `post_checkout`
    job skips the build unless `docs/` changed and a `definitions/` change would then stop
    rebuilding the pages it produces. The churn is collapsed in GitHub diffs by
    `linguist-generated`, which R7.1.8 put into `.gitattributes`.

<a id="r7-4"></a>
**R7.4** *(phase R7, **after R8.5**; deferred 2026-09-19)* Convert `trackerlog.tex` into
    `CHANGELOG.md`.

    **Moved behind R8.5 deliberately.** R8.5 migrates `trackerlog.txt` to one JSON file per
    issue and adds a `resolvedInVersion` field for exactly this purpose — its own text says it
    *"makes R7.4's CHANGELOG.md a pure rendering job instead of a second mechanism"*. Writing the
    converter now means writing it against the comma-escaped flat file first and against JSON
    afterwards, i.e. writing it twice.

    Note that #2544 fixed the LaTeX escaping of issue text in the meantime; the RST writer has
    the same question with different characters, and a Markdown writer will have it again with a
    third set. One escaping decision per output format belongs in the rendering job, not spread
    over three writers.

<a id="r7-5"></a>
**R7.5** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r7-5) — *(phase R7)*
    **The installation documentation says what is shipped** (#2388): Python 3.10-3.14, 64-bit
    only, one wheel-name rule instead of three examples from 2020.


<a id="r7-6"></a>
**R7.6** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r7-6) — *(phase R7; approved
    2026-09-18)* **Every generated GitHub link pointed at a tree that no longer exists** (#2525).
    `main/pythonDev/` became `python/` in R3.1, R3.8 and R3.9; four generators built the URL by
    hand. One prefix in `generatorPaths.py` now, and 368 documentation files repaired.

<a id="r7-7"></a>
**R7.7** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r7-7) *(phase R7; after R7.1.5)*
    **Document the results monitor and the
    package command line** (#2559). Both changed under the documentation's feet in R6.9 and R8.8.

    - **The monitor.** `docs/theDoc/theDoc.tex` still says *"copy `resultsLoader.py` to your
      directory and call `python resultsMonitor.py file.txt`"* and pastes a `-h` output that no
      longer exists. It is rewritten as Markdown in `docs/manual/`: the three ways to call it (the
      package command line, `python -m exudyn.misc.resultsMonitor`, and `MonitorResults(...)` from
      a script), the file dialog and `--last`, the control panel, the settings file, and a
      **pointer to `--help` instead of a pasted option list** that goes stale the next time an
      option is added.
    - **The command line.** `python -m exudyn` is documented nowhere, because it did not exist. It
      needs a page of its own with the four commands, and `info` named in `CONTRIBUTING.md` as the
      thing to paste into a bug report.

    **Correction to R7.1.5 while doing it**: `theDoc.tex` is *not* purely the LaTeX skeleton, as
    that step assumed. Besides the preamble it carries the section headers that wrap the generated
    chapters (`sec:pythonUtilityFunctions`, `sec:item:reference:manual`, `sec:settingsStructures`,
    `sec:issueTracker`, `License`) and this one hand-written section. R7.1.6 and R7.1.7 must not
    delete it without moving those.

## R8 — Process  <!-- old Phase 7 -->

**The order of this phase** (maintainer question 2026-09-21: *"8.2 would fit better after the
    revision of the issue tracker file format"* — yes, and the same holds for more than R8.2).
    Step numbers are permanent; this is the order they are **worked** in:

    | # | step | why here |
    |---|---|---|
    | 1 | **R8.3** (+R8.3.1 remainder, **R8.3.3**) | the CLI is what every later step is driven from; it also removes the cwd and Windows-path dependency that blocks scripting and CI |
    | 2 | **R8.5** (+**R8.5.2**, then R8.5.1) | the file format. Everything that writes or reads an issue is cheaper to write once against JSON than twice |
    | 3 | **R8.3.4** | `ABANDONED` — `CLOSED` (D13): a rename that every later step would otherwise carry twice |
    | 4 | **R8.4** | the minor-version bump belongs in the tracker, and its baseline should land in the new format, not in a Python list; **and (b)**, the stored version of a closed issue with its check (D14), which the maintainer asked for on 2026-09-21 |
    | 5 | **R7.4** | `CHANGELOG.md` is a rendering of the JSON; the step already says it waits for R8.5 |
    | 6 | **R8.1** | templates and `CONTRIBUTING.md`; independent of all of the above, can be pulled forward whenever it suits |
    | 7 | **R8.2** | `tools/release.py` drives the tracker CLI (1), the minor bump (3) and the JSON (2). Written before them it is written twice — which is the maintainer's point |
    | 8 | **R7.2** | carry the revision into the documentation, with the R8 changes included (maintainer, 2026-09-21) |
    | 9 | **R8.6** | the user-script checker needs the complete API-changes table, so it stays last |

    R9 to R11 follow the documentation pass (maintainer, 2026-09-21).

<a id="r8-1"></a>
**R8.1** Issue and PR templates, and a `CONTRIBUTING.md` stating the actual policy now that a branch
    exists to target.

<a id="r8-2"></a>
**R8.2** `tools/release.py`: bump → regenerate → test → build → tag.

<a id="r8-3"></a>
**R8.3** **DONE 2026-09-21** (#2567) → [log](exudynRevisionLog2026.md#r8-3) — *(phase R8)*
    **The issue tracker gets a command line** — **in `exudev`**, not in a second entry point
    (maintainer 2026-09-21): `exudev issue raise | extend | remark | resolve | abandon | show |
    list | modify | mode`. It was driven by importing the module from its own directory and
    calling functions, which is why every issue of this revision was raised with a four-line
    `python -c`.

    **Done**: the cwd dependency and the Windows-separator literals are gone (`trackerDirectory`
    and `repositoryRoot` from `__file__`); every writing verb is a `Step` with an `action`, so
    `exudev -n issue resolve ...` prints what it would do and writes nothing; `mode
    --release/--dev` rewrites the one `versionDev` line of fact 26 and reruns the update;
    `list` filters by status, type, effort and priority, which is what R8.5.2's triage pass needs;
    eight tests cover the version arithmetic (`ResolvedIssues2Version`,
    `GetMajorMinorMicroVersion`, `VersionString`) and the CLI.

    **Two items of this step were void when it ran**: the `NORMAL`-vs-`med` priority mismatch was
    settled in R8.5.3, and `execWithPythonVersion.bat` — whose stale
    `cd ..\tools\makeWindowsBinaries\` this step wanted fixed — exists only in the untracked
    `tmp/oldScripts/` since exudev replaced the batch files in R5.18.

    **Still open**: a web mask, which is R8.5.1 (`exudev issue serve`) and comes after the JSON
    format, because a viewer that edits issues should edit the format that stays.

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

    **The status itself was built in R8.7** (2026-09-18) — as `ABANDONED`, with `AbandonIssue()`
    and a mandatory reason; the **`close` verb landed with the CLI of R8.3** (2026-09-21, as
    `exudev issue abandon`, alias `close`). The JSON schema of R8.5 carries it since 2026-09-21.

    **The name is wrong and is corrected to `CLOSED` (maintainer, 2026-09-21).** This step first
    suggested `CLOSED`, R8.7 built `ABANDONED`, and the two have stood side by side since — which
    is the whole problem, because they do not mean the same thing. *Abandoned* is one **reason**
    among several; the **status** is that the issue is closed and was not resolved. Obsolete,
    won't fix, duplicate, superseded, not reproducible and abandoned are all that same status.

    So: **`RAISED`, `RESOLVED`, `CLOSED`** — and `CLOSED` means *everything except RESOLVED*,
    with the kind named in the mandatory reason, which is what the release notes and a reader see.
    Naming the kinds as statuses was rejected in this step for a reason that still holds: the
    distinction is prose, and every extra status is another branch in every converter.

    **Still to do** (sub-step R8.3.4): `ABANDONED` -> `CLOSED` in `issueStatuses`,
    `closedStatuses`, the 15 issue files that carry it, the converters and the tests;
    `AbandonIssue` -> `CloseIssue` with `AbandonIssue` kept as a one-line alias; `exudev issue
    close` as the primary verb and `abandon` as its alias (it is the other way round today); and
    the description of the status in `issueTracker.py` listing the kinds it covers, so that the
    next reader does not invent a second status for one of them.

    **And the constraint stated above does not hold; the opposite does.** "A closed-not-fixed issue
    must not count as resolved" sounds right and is unimplementable: the micro version is a running
    count, so if abandoning did not count, then abandoning an issue that is already RESOLVED - which
    is exactly what a migration of the 2019-2022 backlog does - moves `version.txt` **backwards**,
    and a released version number stops being reproducible from the file. The rule is therefore
    that the version counts **closed** issues, resolved and closed-not-resolved alike, and the
    distinction lives where it is actually read: the release notes list such an issue neither as
    resolved nor as open. Measured: 15 issues moved to that status and `version.txt` stayed at
    1.11.178.

    **Confirmed by the maintainer on 2026-09-21 and verified against the released history**, which
    is the part that was open: a closed issue **must** count for the micro version, or correcting
    an old issue renumbers versions that are already published. It does count, and nothing moved
    — the archived `trackerlog.rst` of 1.11.0 and the current `trackerlog.md` agree on the version
    of **all 2,054 issues both of them name**, 0 differences (fact 31). The 12 issues that were
    RESOLVED in the released history and are closed-not-resolved today still count; they only
    left the printed *resolved* list, which is why 1.9 ends at a printed 1.9.234 while its last
    micro was 1.9.235 (#1959).

<a id="r8-3-3"></a>
**R8.3.3** **DONE 2026-09-21** (#2566) — *(sub-step of R8.3; maintainer request 2026-09-21)* **An issue can be extended.** The
    tracker can raise an issue and close it, and `ChangeIssue` can overwrite a single field; what
    it cannot do is the thing that actually happens: the first analysis of a problem turns up
    more, and that belongs **with** the issue, not in a second issue and not by rewriting the
    description someone else wrote.

    `ExtendIssue(issueNumber, text, author=...)` appends a dated paragraph to the description and
    touches nothing else — not the status, not the type, and above all not the version, which is
    derived from the count of closed issues. In the flat file it is one field to append to; in the
    JSON of R8.5 the same call writes one entry of an `updates` list, which is what makes the
    history readable afterwards.

    **DONE 2026-09-21** (#2566): `ExtendIssue()` with its tests, and the CLI verb
    `exudev issue extend <n> "text"` with R8.3. What was written here first as a rule about
    `notes` became the two fields of R8.5.3: `ResolveIssue` and `AbandonIssue` write
    `releaseNotes` and clear `workingRemarks`.

<a id="r8-3-2"></a>
**R8.3.2** **DONE 2026-09-17** → [log](exudynRevisionLog2026.md#r8-3-2) — *(sub-step of R8.3;
    maintainer request 2026-09-17)* **The old backlog was checked against what the revision actually
    did.** 29 issues from 2016-2026 were read and verified one by one; 23 are resolved by later
    work, 2 were partially covered and got successors (#2497, #2498), 6 stay open because nothing
    has been done about them.

<a id="r8-7"></a>
**R8.7** **DONE 2026-09-18** → [log](exudynRevisionLog2026.md#r8-7) — *(phase R8; maintainer
    request 2026-09-18)* **One list of issue types and one list of statuses, and the tracker
    enforces them** (#2519). 39 spellings became 10 types; `IMPROVEMENT` is new; `WORK` and
    `TESTING` leave the status list and `ABANDONED` joins it.

<a id="r8-3-4"></a>
**R8.3.4** *(sub-step of R8.3.1; maintainer decision 2026-09-21)* **`ABANDONED` becomes `CLOSED`.**
    One status for every issue that is closed and was not resolved, with the kind of closing in
    its mandatory reason: obsolete, won't fix, duplicate of #n, superseded, not reproducible,
    abandoned. See D13 and the discussion in R8.3.1.

    Touches: `issueStatuses` and `closedStatuses` in `issueTracker.py` and the description that
    lists what the status covers; the **15 issue files** that carry `ABANDONED` today;
    `AbandonIssue` — renamed `CloseIssue`, with `AbandonIssue` kept as a one-line alias for
    scripts; `exudev issue close` as the primary verb (`abandon` becomes the alias, it is the
    other way round today); the converters, `checkIssues.py`, the tests, `docs/dev/WORKFLOW.md`
    and CLAUDE.md rule 3.

    **The version must not move**: the count of closed issues is unchanged by a rename, and the
    check of R8.4(b) is what says so afterwards.

<a id="r8-3-5"></a>
**R8.3.5** **DONE 2026-09-21** (#2570) — *(sub-step of R8.3.1; maintainer question
    2026-09-21: "I assume that an according WARNING appears and something like 'Are you sure'?")*
    **Changing a field of a CLOSED issue is not silent any more.** It was: `ChangeIssue` wrote
    any field of any issue without a word, and so did `exudev issue modify`. Raising, resolving
    and closing are ordinary work; editing an issue whose text stands in the release notes of a
    **released version** is not, and two of its fields decide the version number itself.

    - a **CLOSED issue is refused** unless the caller says so: `force=True`, `--force`. The
      message says what it would change and offers the alternative (raise a new issue).
    - **`status`, `number`, `dateRaised` and `dateResolved` are refused always**, force or not:
      the tracker writes them in `RaiseIssue`, `ResolveIssue` and `AbandonIssue`, and `status`
      is what moves an issue between `open/` and `closed/` and with it the micro version.
    - replacing a **text field that is not empty** prints a warning and the previous text, so
      that it can be pasted back: `modify` overwrites, `extend` and `remark` append.
    - the page of R8.5.1 now shows the editors for a closed issue too — and asks with a
      confirm dialog before it sends `force`. A page that sent it silently would be worse than
      no protection, because the command line refuses the same edit.

    Not done here, because it belongs to R8.4(b): making a closed issue verifiable rather than
    only protected, by storing the version it produced (D14).

<a id="r8-4"></a>
**R8.4** *(phase R8; extended by the maintainer 2026-09-21)* **Fold minor-version bumps into the
    tracker, and make the version numbering verifiable instead of brittle.**

    **(a) The minor bump.** A 1.11 → 1.12 bump currently means hand-editing the `versionResolved`
    list and the `versionNames` dict inside `issueTracker.py`. Make it a command that records the
    baseline automatically. Keep it an explicit maintainer action, never automatic.

    **(b) The version of a closed issue is STORED, not recomputed** (maintainer, 2026-09-21; D14).
    Today every version number in the release notes is derived on each run by sorting the closed
    issues by `dateResolved` and counting down from the current count against the `versionResolved`
    baselines. That means **one corrected date, one status change or one missing file renumbers
    versions that have been published** — which must never happen and which nothing would report.

    The field is already in the schema and unused: `resolvedInVersion`. Write it when an issue
    closes, and then:

    - **backfill it for the history** from the archived `docs/RST/trackerlog.rst` of 1.11.0
      (`tmp/trackerlog.rst`), which states the version of 2,066 resolved issues, and by
      recomputation for the rest — one-shot, like the migration of R8.5;
    - **check it in `tools/checkIssues.py`**: the recomputed version of every closed issue must
      equal the stored one, and the highest stored micro of each minor must equal what the
      baselines say. Two numbers that are derived the same way twice are not a check; a number
      written once and recomputed later is one.
    - **check that no issue number is missing.** Numbers are consecutive and never reused, so a
      gap means a file was lost, and with it one count of the micro version. A missing *open*
      issue at the end of the sequence is harmless and may be erased; a gap is not.
    - a closed issue is then **immutable in the two fields the version depends on** (`status`,
      `resolvedInVersion`): changing one has to change the other, deliberately, which is the
      point of storing it.

    **(c) `versionResolved` stops being a hand-maintained list of magic numbers**: with (b) each
    baseline is derivable from the issues themselves (the count at which the minor changed), so
    the list becomes data the bump command writes and the check verifies.

<a id="r8-5"></a>
**R8.5** **DONE 2026-09-21** (#2568) → [log](exudynRevisionLog2026.md#r8-5) — *(phase R8, after
    R8.3)* **Migrate `trackerlog.txt` to one file per issue, in JSON.**

    **Done**, with two deviations from the text below, both decided with the maintainer on
    2026-09-21: the files live in **`tools/issueTracker/issues/`** and not in `docs/dev/issues/`
    — the tracker is a tool with a command line and its data belongs beside it, while `docs/`
    is documentation (D10) — and the second directory is called **`closed/`** rather than
    `resolved/`, because an ABANDONED issue is closed as well and counts for the version. The
    field names became camelCase in the same move (`dateRaised`, `resolvedAuthor`, `title`), so
    that the new fields and the old ones read alike.
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
    deliberate exception**: the rendered list the documentation shows. That is
    `docs/generated/trackerlog.md` since R7.1.6; it stays committed and stays generated, because
    that is what ReadTheDocs renders (maintainer, 2026-09-17). The LaTeX and RST twins disappeared
    with the LaTeX documentation in R7.1.6 and R7.1.7.

<a id="r8-5-3"></a>
**R8.5.3** **DONE 2026-09-21** (#2566) — *(sub-step of R8.5; maintainer request 2026-09-21)*
    **`notes` was two fields.** For a closed issue it is the **release note**, published in
    `docs/generated/trackerlog.md` and, with R7.4, in `CHANGELOG.md`. For an open issue what is
    worth writing down is something else entirely — *duplicate of #2134*, *marked for
    deprecation*, *check whether this still happens*, *part A solved, B open* — and it is
    worthless the moment the issue closes.

    The column is therefore `releaseNotes`, a new column `workingRemarks` holds the second kind
    and is **cleared when the issue closes**, and `effort` is the third new column (R8.5.2). The
    schema went 14 — 16 columns **before** the CLI of R8.3 and the JSON of R8.5 are written
    against it, so that both meet the final names; it is done on the flat file because the
    migration is unambiguous today and will not stay so (maintainer decision 2026-09-21).

    **Measured before touching anything**: of the 2,567 issues, **629 have a note and not one of
    them is open** — so every existing note is a release note, and the two new columns start
    empty everywhere. `migrateSchema.py` rewrites the file and proves itself: it reads every
    issue before and after and compares them field by field, accepting only the two new empty
    columns and a normalized priority. The version does not move (no issue changes status).

    Also here, because the file was being rewritten anyway: the **nine** priority spellings
    (`''`, `NO`, `NORMAL`, `high`, `med`, `low`, `HIGH`, `medium`, `LOW`) over 251 issues become
    `LOW` / `NORMAL` / `HIGH` or empty, enforced from now on in `RaiseIssueDict` **and** in
    `ChangeIssue`, which wrote any value until now. `nHeaderLines` is read from the header marker
    rather than assumed, and the literal `nLine > 9` in the HTML writer is gone.

<a id="r8-5-2"></a>
**R8.5.2** **DONE 2026-09-21** — [log](exudynRevisionLog2026.md#r8-5-2) — *(sub-step of R8.5;
    maintainer request 2026-09-21)* **The open backlog becomes sortable: an `effort` field, and a
    place for what the work knows.** Measured 2026-09-21: **270
    open issues** — 152 EXTENSION, 33 CHECK, 20 FIX, 19 DOCU, 16 TESTING, 14 CHANGE, 8 BUG, 4
    EXAMPLE, 3 IMPROVEMENT, 1 IDEA — raised between 2019 and today, and **244 of them carry no
    priority at all** (the remaining 26 are spelled six different ways: `high`, `HIGH`, `med`,
    `NORMAL`, `low`, `LOW`). A list in that state cannot be prioritized, only read.

    **`effort`**, one enum, in human working hours without AI assistance:

    | value | hours |
    |---|---|
    | `LOW` | within 2 |
    | `MEDIUM` | within 16 |
    | `HIGH` | within 40 |
    | `HUGE` | above 40 |

    It is a *classification*, not an estimate to be held to: what it buys is "show me every open
    FIX that is LOW" — the list a maintainer can actually work from.

    **Where that kind of information goes** is `workingRemarks`, built in R8.5.3 together with the
    `effort` column: it holds what the work knows while the issue is open and is cleared when it
    closes, so it can never reach the published release notes.

    **Done 2026-09-21.** The tracker side: the field and its enum (R8.5.3), the `--effort` filter
    of `exudev issue list`, and `exudev issue triage`, which prints the open issues as a table of
    type against effort. The pass itself: **269 of the 270 open issues classified** from their
    title and description — 49 LOW, 159 MEDIUM, 54 HIGH, 7 HUGE — and the 270th, #2548, turned
    out to be finished by R7.1.7 and was resolved instead. **These are proposals**: a
    classification made from the text of an issue, not by the person who will do the work, and
    `exudev issue modify <n> effort <value>` corrects one in a second.

<a id="r8-5-1"></a>
**R8.5.1** **DONE 2026-09-21** (#2569) — [log](exudynRevisionLog2026.md#r8-5-1) — *(sub-step
    of R8.5)* **A tiny local viewer/editor for the issues**, for maintainers:
    **`exudev issue serve`** (not `python tools/issueTracker/serve.py`: the tracker has one
    command line since R8.3 and this is a verb of it) opens a local web page - **stdlib `http.server` and one
    HTML page, no new dependency** (a Qt6 front-end would cost PySide6, against rule 6, and a web
    page also works over SSH). Features: list by id, search, filter open / resolved / both, and
    **RaiseIssue, EditIssue, ResolveIssue** writing through the same API the scripts use.
    **Deleting an issue stays manual and file-based** — it should be rare (a wrongly raised issue)
    and deliberate.

    **Done 2026-09-21** as `tools/issueTracker/issueServer.py`, 12 tests, and with one addition the
    text above does not name: the server binds **127.0.0.1 only and checks the `Host` header**, so
    that a page in this browser cannot reach the store through a rebound name. The store is the
    version of the package and the server has no authentication.

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

<a id="r8-8"></a>
**R8.8** **DONE 2026-09-19** *(maintainer session; integrated 2026-09-20)* **The installed package
    gets a command line: `python -m exudyn <command>`** (#2558). `monitor`, `plot`, `info`, `demo`,
    in a plain dispatch dictionary `CommandTable()`, each command imported when it is called.
    `info` prints what a bug report needs: version, `config.Version(True)`, package path,
    `EXUDYN_MODULE`, output directory, Python, platform and which of numpy, scipy, matplotlib,
    networkx, ngsolve and pytest are installed.

    **Deliberately not a console script**: nothing goes on `PATH` until the command set has settled,
    so `pyproject.toml` is untouched. The dispatch dictionary is the extension point for **R9.6**,
    which can add plugin commands through `importlib.metadata` entry points.

    This is the user-facing counterpart of `tools/exudev`, which stays the maintainer driver and
    keeps its rule of never importing exudyn.

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
<a id="r11-4"></a>
**R11.4** **DONE 2026-09-20** (all five sub-steps) *(maintainer, 2026-09-19; before R7 is
    finished)* **The layout tasks that the
    documentation depends on.** A deliberate interruption of the documentation phase: *"at least the
    implementation files part will be needed for documentation on how to add new Exudyn items"* —
    a how-to that names `checkPreAssembleConsistencies.cpp` and `VisuNodePoint.cpp` as they are today
    would have to be rewritten a month later. Five sub-steps, each with its own issue and commit.

<a id="r11-4-1"></a>
**R11.4.1** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r11-4-1) *(sub-step of R11.4)*
    **Five modules move into `exudyn/misc/`** (#2552): `docmeta.py`,
    `GUI.py`, `resultsMonitor.py`, `extensionRegistry.py`, `mainSystemExtensions.py`. None of them
    is a modelling module: two are machinery, two are tools, one is the extension mechanism.

    **Relocation only.** `resultsMonitor.py` is being revised in another session at the same time,
    so its content is not touched here and the references to it are fixed when those changes are
    integrated (maintainer, 2026-09-19).

<a id="r11-4-2"></a>
**R11.4.2** **DONE 2026-09-19** → [log](exudynRevisionLog2026.md#r11-4-2) *(sub-step of R11.4)*
    **Remove the comment *"only import if it does not conflict"***
    (#2553). It stands in **458 tracked files** behind `import exudyn.graphics as graphics`. It was
    meant to warn that the name can collide with another package or a local variable; it reads as a
    condition on the import. The import stays, the comment goes. The copies under
    `docs/RST/Examples` and `docs/RST/TestModels` are generated and follow by regeneration.

<a id="r11-4-3"></a>
**R11.4.3** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r11-4-3) *(sub-step of R11.4)*
    **Split `checkPreAssembleConsistencies.cpp`** (#2554). Measured while doing it: **49** functions
    in **four** kinds, not 62 in five — `MainLoad` has no such check. 2,602 lines became
    `checkPreAssembleConsistencies{Objects,Markers,Nodes,Sensors}.cpp` plus a header for what
    they share. **One file per kind and not one per item**: each includes pybind11, and that
    include dominates the compilation time.

<a id="r11-4-4"></a>
**R11.4.4** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r11-4-4) *(sub-step of R11.4)*
    **The `UpdateGraphics` functions leave `VisuNodePoint.cpp`**
    (#2555): ~79 implementations for every node, object, marker, load and sensor in 4,171 lines,
    under a name that only made sense when `VisualizationNodePoint` was the first item to have one.
    Same split as R11.4.3, into `Visu<ItemType>.cpp`.

    **To be measured first** (maintainer): whether each `UpdateGraphics` can instead go into its own
    `CItem*.cpp`, which is where it belongs. The question is compilation time, because pybind11 is
    not included in most of those files today. If the measurement says it is affordable, that is
    what happens instead.

<a id="r11-4-5"></a>
**R11.4.5** **DONE 2026-09-20** → [log](exudynRevisionLog2026.md#r11-4-5) *(sub-step of R11.4)*
    **`src/Objects/` becomes three
    folders** (#2556): `ImplObjects`, `ImplNodes`, `ImplMarkers`. Loads and sensors are one short
    file each and stay in `System`; the `Visu<ItemType>.cpp` files go to `Graphics`, and
    `checkPreAssembleConsistencies*.cpp` and `evaluateUserFunctions.cpp` to `System`, where they
    belong.

    The maintainer's own framing, kept because it is the decision: *"the folder structure is
    certainly not very systematic, but I would rather vote to stay with the smaller changes rather
    than restructuring everything."* Every file here is a tracked file, so the move needs approval
    when the step runs; `msvc/cppsrc.vcxproj`, `sources.json` and `tools/gen_sources.py` follow.

