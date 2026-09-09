# Development workflow

How work is done on this repository: issue tracker, versioning, and the gates a commit must pass.
Written for Claude Code sessions, but it describes the human workflow too.

## 0. Environments

Python is **not** on `PATH` under a plain shell. Use the named conda environments:

| purpose | interpreter |
|---|---|
| **default — generators, docs and tests** | `C:\Users\c8501009\Anaconda\envs\venvExuP313\python.exe` |
| **per-version test matrix** | `...\envs\venvP31x\python.exe`, x in 0–4 |

`venvExuP313` carries Exudyn, scipy, ngsolve/h5py and the full sphinx toolchain, so one environment
covers regeneration, the docs build and the test suite. Its recipe and the package-to-feature
mapping are in [`docs/howTo/condaEnvironments.md`](../howTo/condaEnvironments.md). The batch
scripts in `tools/buildAndGenerate/` select environments themselves via
`execWithPythonVersion.bat` and `execWithAllPythonVersions.bat` (P310–P314).

> **Check before trusting a test run.** `import exudyn; exudyn.__version__` must match
> `docs/theDoc/version.txt`. The **base** Anaconda environment carries a stale **1.10.0**, so a
> test suite run from it silently tests old binaries.

Run the generators and the docs build from a Windows shell (PowerShell). Until plan step 70 adds a
`.gitattributes`, generating from a different shell can still produce spurious line-ending diffs.

Documentation build, from the repository root:

```bash
sphinx-build -b html . _build -E
```

### Dependencies

**numpy is the only hard requirement**, and that follows from the nature of the C++/Python
coupling — it is not a preference and should not grow.

Advanced functionality legitimately needs more: scipy, networkx, Gym, stable-baselines, NGsolve
and others. Those stay **optional**. Installing exudyn requires the minimum; a function that needs
more raises at the point of use. Today those failures are not consistently `ImportError`, which is
a known rough edge and a Phase 5 concern (steps 46–49) — not something to fix opportunistically.

When adding code: a new *optional* dependency behind a clear failure is acceptable; a new
*mandatory* one is not.

## 1. The issue tracker

`tools/issueTracker/issueTracker.py` is both the issue tracker **and the source of truth for the
version number**. Data lives in `tools/issueTracker/trackerlog.txt` (2351 issues, 278 open as of
1.11.0).

Run it from its own directory — it uses relative and Windows-style paths:

```bash
cd tools/issueTracker
```

### API

| call | effect |
|---|---|
| `RaiseIssue(issueName, description, issueType='EXTENSION', fileName='', lineNumber='', deadline='', author='JG', priority='')` | appends a `RAISED` issue; default deadline +180 days |
| `RaiseIssueDict(issueDict)` | same, full control over fields |
| `ResolveIssue(issueNumber, notes='', author='JG')` | marks `RESOLVED`, stamps date, **bumps the micro version** |
| `ChangeIssue(issueNumber, key, value)` | change one field |
| `ModifyDictIssue(issueDict)` | replace a whole issue (needs `number`) |
| `GetIssue(n)` / `GetIssues()` / `NumberOfIssues()` | read-only |
| `VersionString()` / `GetMajorMinorMicroVersion()` | current version |

Fields: `number, issue, author, status, description, type, priority, date raised, deadline,
date resolved, resolved author, file, line, notes`.

- `status`: `RAISED`, `WORK`, `TESTING`, `RESOLVED`
- `type`: `BUG, FIX, NEW FEATURE, EXTENSION, CHANGE, PERFORMANCE, IDEA, CHECK, CLEANUP, DOCU,
  TUTORIAL, TESTING, EXAMPLE, DISCUSSION`
- `priority`: `''`, `LOW`, `MED`, `HIGH`

> **Never hand-edit `trackerlog.txt`.** Text fields may not contain a literal `,`; the tool escapes
> it to `\;` on write and unescapes on read. Editing by hand corrupts the column count and every
> reader then reports "inconsistent line definition".

### Known inconsistencies (raise as issues; do not fix inline)

- The header of `trackerlog.txt` documents `priority` values `NO, LOW, NORMAL, HIGH`, but
  `ConvertToHTML` colours on `high` / `med` / `low` and prints *"priority undefined"* for anything
  else. `NORMAL` therefore warns.
- Historical `type` values include typos and variants outside the documented set: `EXTENSON`,
  `CHEKCK`, `Extension`, `TEST`, `OPTIMIZE`.
- `execWithPythonVersion.bat` ends with `cd ..\tools\makeWindowsBinaries\`, a directory that no
  longer exists — the folder is now `tools/buildAndGenerate/`.

### What next?

Answer in this order:

1. Open `BUG`s (`status != RESOLVED and type == 'BUG'`) — currently 9.
2. Open issues by priority (`HIGH`, then `MED`, then `LOW`) — currently 9 / 5 / 4.
3. The current phase of `docs/revision/exudynRevisionPlan2026.md`.

```bash
cd tools/issueTracker
C:/Users/c8501009/Anaconda/envs/venvP312/python.exe -c "import issueTracker as it; [print(i['number'], i['priority'], i['type'], i['issue']) for i in it.GetIssues() if i['status'].strip()!='RESOLVED' and i['priority'].strip().lower()=='high']"
```

## 2. Versioning

**The micro version is derived, not written.** `GetMajorMinorMicroVersion()` counts `RESOLVED`
issues and subtracts the baseline for the current minor version, so **resolving an issue *is* the
version bump**. `ResolveIssue()` then rewrites, in one pass:

```
tools/issueTracker/trackerlog.txt          (+ trackerlog_backup.txt)
tools/issueTracker/trackerlog.html
docs/theDoc/trackerlog.tex
docs/RST/trackerlog.rst
main/src/Autogenerated/versionCpp.cpp      the version reported by exudyn at runtime
docs/theDoc/version.txt
docs/theDoc/versionName.txt                the jazz-musician release name
```

Never edit any of those seven files by hand.

**Minor bumps (1.11 → 1.12) are manual and are the maintainer's decision.** They require editing
two constants in `issueTracker.py`: append the current total resolved count as `version12xResolved`
to the `versionResolved` list, and add the release name to `versionNames`. Claude asks first, every
time. Plan step "issueTracker CLI" folds this into the tool later.

## 2a. Branches and remotes

Full picture in plan §2a. The short version, which is what matters day to day:

| branch | purpose |
|---|---|
| `master` | read-only mirror of public GitHub `master`, frozen at 1.11.0 — **never commit here** |
| `v2-dev` | all v2.0 work; the working branch |
| `release/*` | release preparation |

**`origin` currently points at GitHub, and `master` tracks it.** The internal server does not exist
yet, so the names stay as they are until the first internal sync (step 7) — renaming the only real
remote before then buys nothing. Consequence: a bare `git push` from `master` would publish to the
public repository.

Therefore, until step 7 and step 9's `pre-push` hook are in place:

- Work on `v2-dev`; never commit on `master`.
- **Do not push at all.** Claude never pushes to any remote, under any circumstances.

## 3. Commit tiers

| tier | when | gates |
|---|---|---|
| **sync** | `v2-dev` only, knowingly non-working, to move work between machines | none — but the message must be prefixed `WIP:` and say what is broken |
| **normal** | the default for every real change | all four gates in §4 |
| **release** | a minor version advance, i.e. an Exudyn *Release* | §4 plus the full `Examples` run, the wheel matrix across P310–P314, and maintainer sign-off |

## 4. The four gates for a normal commit

Run in this order; stop at the first failure.

### 1. The build succeeds

Required for any change to C++, `main/setup.py`, or `main/obj/cppsrc.vcxproj`. VS2022
`Debug|x64` or `Release|x64` from `main/main_sln_Template.sln`, or
`tools/buildAndGenerate/buildInstallSingleVersion.bat`.

### 2. Regeneration is clean

On a clean tree, run all six generators from `main/src/pythonGenerator/`:

```
pythonAutoGenerateObjects.py
pythonAutoGenerateSystemStructures.py
autoGeneratePyBindings.py
utilitiesDocuGenerator.py
createStubFiles.py
doc2rst.py
```

`makeAllBinariesScripts.py` is **not** part of this — it writes only a volatile build date.
Then `git status --porcelain` on the Tier 1 paths (plan §4.2) must be **empty**; Tier 2 paths
(plan §4.3) should be empty and otherwise warn. Most generators skip unchanged files
(`WriteTextIfDifferent`, ignoring `@date` and `last modified` lines), so `git status` is a usable
drift signal. Once plan step 2 lands, this is `tools/regenerate.py`.

Two measured caveats (2026-09-09, plan §3 facts 11 and 13):

- `main/src/Autogenerated/pybind_manual_classes.h` is rewritten unconditionally by
  `autoGeneratePyBindings.py`, so its `// AUTO:  last modified` line **always** shows as modified.
  Ignore that one line; treat any other change in it as real drift.
- The committed documentation at `e44aca1` is **already** out of sync with
  `main/pythonDev/Examples/` — a fresh regeneration changes 50 doc files and adds 6. Until that is
  reconciled, compare against a fresh regeneration, not against the committed state.

### 3. The full test suite passes

```bash
cd main/pythonDev/TestModels
C:/Users/c8501009/Anaconda/envs/venvP312/python.exe runTestSuite.py -quiet
```

About 22 s serially — far shorter than the compile — so it runs **in full, before every commit,
never as a subset**. Record the failed-test count and compare it against the run before the change.

Two things that will otherwise look like breakage:

- **Pin scipy to 1.15.2.** scipy 1.18.0 slows the suite from ~22 s to over 10 minutes, apparently
  in the eigensolver path (plan fact 19). If a run suddenly takes minutes, check the scipy version
  before looking for a regression in Exudyn.
- **The global tolerance is 5e-14 and some models sit close to it.** A failure just above it — for
  example `movingGroundRobotTest.py` at `5.0688e-14`, with result and reference agreeing to ~13
  significant digits — is floating-point noise from a different numpy/BLAS build, not a real
  break (plan fact 20). Compare the reported `RESULT` and `refsol` before treating it as one.
  Per-model tolerances are plan step 40.

**The suite overwrites a tracked file.** `runTestSuite.py` writes
`main/pythonDev/TestSuiteLogs/testSuiteLog_<version>_<platform>-P<x.y>.txt`, which is committed
per release (decision D7). In normal use this is harmless: the suite runs *after* a micro-version
change, so the filename carries the new version and the previous log is kept, not replaced. The
log that matters for a version is therefore the one from **its first micro-version change**.

It does bite when the suite is run *without* a version change — an incidental gate run overwrites
that version's existing log in place. Check the file before committing and keep the release run.
A backup mechanism would remove the hazard entirely (step 72).

`Examples` are **not** run here: they take many minutes and only check that scripts do not crash
(they time out after a few seconds each, and are not compared for identical results). Run them
with `tools/buildAndGenerate/runTestExamples.bat` for releases and large steps only.

### 4. Docs and plan are updated

- `docs/theDoc/*.tex` and the RST sources, wherever user-visible behaviour changed.
- The step status in `docs/revision/exudynRevisionPlan2026.md`. If a fact in plan §3 turned out to
  be wrong, correct it there rather than working around it.

## 5. Committing

After the gates pass:

1. `ResolveIssue(issueNumber, notes='...')` — this bumps the micro version and rewrites the seven
   generated files listed in §2.
2. Present the maintainer with an overview: files changed, gate results (build, drift, test count),
   the issue resolved, and the proposed commit message.
3. **Wait for explicit approval.** Claude commits only on a clear go-ahead.
4. **Claude never pushes, to any remote, ever.** Plan step 9 adds a `pre-push` hook refusing
   pushes to `github` from any ref but `master` / `release/*`; until then the rule is social.

Commit message convention — reuse the tracker's own vocabulary so commits and issues speak the same
language:

```
<TYPE> #<issue number>: <one-line summary>

<why, and anything the diff does not make obvious>

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
```

Example: `BUG #2107: fix 32-byte alignment of VectorBase under AVX2`

## 6. Build and release scripts

`tools/buildAndGenerate/` (Windows batch; never pushed to GitHub so far):

| script | purpose |
|---|---|
| `runPythonScripts.bat` | run the generators (everything except the system-structures update) |
| `runTestSuite.bat` | test suite, one or all Python versions |
| `runTestExamples.bat` | the Examples set — slow, releases only |
| `runPerformanceTests.bat` | performance suite |
| `buildInstallSingleVersion.bat` | wheel + msi for one Python version, uninstall and reinstall |
| `makeWindowsBinaries.bat`, `makeInstallBinaries.bat`, `makeAndTestAllBinaries.bat` | the full Windows binary path |
| `makeUbuntuWheels.bat`, `makeUbuntuManyLinuxWheels.bat`, `manylinuxBuild.sh` | Linux wheels |
| `makeDoc.bat` | assemble the documentation release directory |
| `removeBuildsAndEggs.bat` | clean wheels and build directories |
| `execWithPythonVersion.bat`, `execWithAllPythonVersions.bat` | conda environment wrappers |
