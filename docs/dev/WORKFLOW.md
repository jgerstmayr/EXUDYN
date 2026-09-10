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

> **Author attribution.** When Claude raises or resolves an issue on the maintainer's behalf, pass
> `author='Claude-JG'` to **both** `RaiseIssue` and `ResolveIssue` — the two fields are separate
> (`author` and `resolved author`), so both need it. `JG` alone means Johannes worked it himself.
> This keeps the tracker honest about who did what without needing a separate audit trail.

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

## 0a. One-time setup per clone

```bash
git config core.hooksPath tools/hooks
```

This activates the tracked hooks in `tools/hooks/`, currently `pre-push`, which refuses to push
anything but `master`, `release/*` and tags to the public GitHub repository. **Git hooks are not
themselves version controlled** — `.git/hooks/` never travels with a clone — so this setting is
per-clone and easy to forget. Without it there is no mechanical guard against publishing `v2-dev`.

Also set `receive.shallowUpdate true` on the *internal server* repository. This clone is a
`--depth 1` shallow clone, and a push from a shallow clone is rejected by default with
`shallow update not allowed`. With that setting the push succeeds and the result is sound —
verified with `git fsck` on both the bare repository and a fresh clone.

## 2a. Branches and remotes

Full picture in plan §2a. The short version, which is what matters day to day:

| branch | purpose |
|---|---|
| `master` | read-only mirror of public GitHub `master`, frozen at 1.11.0 — **never commit here** |
| `v2-dev` | all v2.0 work; the working branch |
| `release/*` | release preparation |

```
origin  →  git@git.uibk.ac.at:c8501009/exudyn.git   internal   (v2-dev tracks origin/v2-dev)
github  →  git@github.com:jgerstmayr/EXUDYN.git     public     (master tracks github/master)
```

`origin` is the internal server, so a bare `git push` from `v2-dev` syncs there — the safe default,
and the normal thing to do. The internal repository carries **full history**; this clone is still
`--depth 1` and stays small.

- Work on `v2-dev`; never commit on `master`.
- Push `v2-dev` to `origin` freely.
- **Nothing reaches GitHub before the v2.0 release.** `tools/hooks/pre-push` enforces this, but
  only where `core.hooksPath` is set — see §0a, and tell anyone you add to the server.
- Claude never pushes to any remote, under any circumstances, and announces network access first.

## 2b. Continuous integration

GitHub Actions only fire on pushes to `master` and on pull requests, so **while `master` is frozen
at 1.11.0 nothing runs there**. `.gitlab-ci.yml` covers the gap.

| leg | how it is covered |
|---|---|
| **Linux x86_64, cp310–314** | `.gitlab-ci.yml`, on the internal GitLab's shared Docker runners |
| **Windows x64** | this development machine; `tools/buildAndGenerate/makeAndTestAllBinaries.bat` weekly |
| **macOS** | a real Mac, at milestones |
| **Linux aarch64** | not covered until GitHub CI resumes — accepted gap |
| **docs** | `docs` job builds sphinx with `-W`; it does **not** deploy |

**Nothing is triggered by an ordinary push.** Pipelines start from the weekly schedule, the
*Run pipeline* button, or a tag. The schedule itself lives in the GitLab UI, not in the file:
**Settings → CI/CD → Schedules**, target branch `v2-dev`. If that schedule is deleted, CI silently
stops and the repository looks exactly the same — worth checking if the pipeline list goes quiet.

The Linux job runs `tools/ci/buildManylinux.sh <pyTag>` **inside** the manylinux image, which is
also what the local docker path uses, so a CI failure reproduces locally with one command:

```bash
tools/buildAndGenerate/makeUbuntuManyLinuxWheels.bat      # all five, via docker + WSL
```

Regular CI sets `EXUDYN_NOFAST=1`, which skips the `__FAST_EXUDYN_LINALG` binary and roughly halves
build time. Ordinary test runs do not exercise that binary. **Release builds must not set it.**

### Reproducible vs sensitive tests

`runTestSuite.py --exit-code` returns non-zero when tests fail — CI depends on this, and without it
CI cannot fail at all. But not every test is reproducible across machines:

- **contact and friction models** are chaotic; a different machine gives a materially different
  error, and the size of that error says nothing about correctness
- **sparse eigenvalue problems** go through ARPACK from a random start vector that cannot be seeded

Those are listed in `SensitiveTests()` in `runTestSuiteRefSol.py`. They still run, and their
failures are reported prominently — but they **do not set the exit code**, because a scheduled run
that goes red at random is an alarm nobody reads. Tests needing a looser but still meaningful
tolerance go in `TestExamplesToleranceFactors()` instead.

`SensitiveTests()` is currently **empty and needs populating from evidence**: run the suite on
Windows, Linux and macOS and compare per-test `ERROR` values; anything varying by orders of
magnitude belongs there. Do not populate it by matching file names — contact/friction/eigen matches
about a third of the suite and would gut the gate.

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

### 2. Regeneration is clean — and runs *after* `ResolveIssue`

> **Order matters.** `ResolveIssue` bumps the micro version and rewrites `docs/theDoc/version.txt`.
> `README.rst` and `docs/RST/Exudyn.rst` **embed** the version string and are refreshed only by
> `doc2rst.py`. Regenerating before resolving therefore always leaves those two files one version
> behind, and the next run reports drift that has nothing to do with your change.
>
> The working order is: **resolve the issue first, then regenerate, then commit.** During
> development, run the generators as often as you like; the run that matters is the last one.

Use the tool:

```bash
python tools/regenerate.py --check
```

It runs the six generators from their required working directory and classifies every difference
against HEAD: **Tier 1** (the API surface) fails with exit 1, **Tier 2** (documentation) warns,
anything outside both tiers is reported as an unexpected gap in the manifest. `--no-run` checks
without regenerating; `--python` selects the interpreter. It ignores modifications outside the two
tiers as unrelated work, and ignores files that differ only in their `@date ... (last modified)`
line — but never excuses a real change inside a tier, however the file got that way.

The manual equivalent, if you need it — on a clean tree, from `main/src/pythonGenerator/`:

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

**Committed logs are protected — the runner diverts rather than overwriting.** All three runners
(`runTestSuite.py`, `runTestExamples.py`, `runPerformanceTests.py`) write a release-named log into a
tracked directory, truncating it at startup *before any test runs*. Since 2026-09-10 they check
first:

- target does not exist → written normally (the release flow: the version was just bumped)
- target exists → the log is **diverted to `main/pythonDev/logsTmp/`** (gitignored), the tests run
  as usual, and a message names the override
- `--overwrite-log` → the existing log is replaced deliberately

Existence is the test, so this also covers the multi-machine case: the committed log from another
machine with the same platform and Python is present, and is not clobbered.

`logsTmp/` is one shared directory for all three runners, so clearing it is a single delete.

### What is in a log

The header records what results depend on: Exudyn version and build date, CPU and core count, and
the installed versions of the relevant packages (`numpy`, `scipy`, `matplotlib`, `ngsolve`, …,
listed as `not installed` when absent). scipy 1.18 vs 1.15 already changed suite runtime by more
than an order of magnitude, so this is not decoration.

The log ends with a per-test overview — one fixed-width line per test with result, error, effective
tolerance and runtime, for both TestModels and MiniExamples, with sensitive tests marked `*`. That
table is how `SensitiveTests()` gets populated: diff two of them from different machines and the
non-reproducible tests stand out.

**Performance logs are per machine.** Set `EXUDYN_MACHINE_ID` once per machine (e.g. `i7-1370P`)
and `runPerformanceTests.py` files its logs in that subfolder, since timings from a laptop and a
workstation are not comparable. Without it, a legacy fallback still routes any 20-core machine to
`i7-1370P/`; that fallback goes away once the variable is set everywhere.

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
   generated files listed in §2. **Then re-run the generators** (at minimum `doc2rst.py`), because
   `README.rst` and `docs/RST/Exudyn.rst` embed the version string — see gate 2.
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
