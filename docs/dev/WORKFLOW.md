# Development workflow

How work is done on this repository: issue tracker, versioning, and the gates a commit must pass.
Written for Claude Code sessions, but it describes the human workflow too.

## 0. Environments

Python is **not** on `PATH` under a plain shell. Use the named conda environments:

| purpose | interpreter |
|---|---|
| **default — generators, docs and tests** | `%USERPROFILE%\Anaconda\envs\venvExuP313\python.exe` |
| **per-version test matrix** | `...\envs\venvP31x\python.exe`, x in 0–4 |

`venvExuP313` covers regeneration, the docs build, the docstring check and the test suite. Its
packages are the dependency groups in `pyproject.toml` (`pip install --group dev`, pip >= 25.1) plus
the `[tests]` extra of the locally built wheel; the recipe is in
[`docs/howTo/condaEnvironments.md`](../howTo/condaEnvironments.md). The
driver `exudev` selects the environment itself – `--py P310`…`P314`, `--py all`, or
`--env NAME` – so nothing has to be activated by hand; see
[`tools/exudev/README.md`](../../tools/exudev/README.md).

> **Check before trusting a test run.** `import exudyn; exudyn.__version__` must match
> `version.txt`. The **base** Anaconda environment carries a stale **1.10.0**, so a
> test suite run from it silently tests old binaries.

Run the generators and the docs build from a Windows shell (PowerShell). Until revision2026 step R0.5 adds a
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
a known rough edge and a revision2026 phase R6 concern (revision2026 steps R6.1–R6.4) — not something to fix opportunistically.

When adding code: a new *optional* dependency behind a clear failure is acceptable; a new
*mandatory* one is not.

## 1. The issue tracker

`tools/issueTracker/issueTracker.py` is both the issue tracker **and the source of truth for the
version number**; `exudev issue <verb>` is its command line. The issues live in
`tools/issueTracker/issues/` as JSON since revision2026 step R8.5 (2,568 issues, 270 open as of
1.11.225):

```
issues/open/2567.json        one file per OPEN issue
issues/closed/2473.json      ... per issue closed since 1 January of the previous year
issues/archive/2019.json     ... per YEAR for the older ones, written once
issues/meta.json             the date of the last change and the version it produced
```

**Every new issue becomes a step in the revision plan** — a new step, or part of an existing step
where it belongs; small issues may share a step. The tracker describes the issue, the plan says
when and together with what it is done (plan §12).

Run it from its own directory — it uses relative and Windows-style paths:

```bash
cd tools/issueTracker
```

### API

**From the command line** (revision2026 step R8.3), which is the way to use it:

```powershell
exudev issue list --open --type FIX --effort LOW    #the list a triage pass works from
exudev issue show 2566
exudev issue raise "title" "description" --type FIX --effort LOW --author Claude-JG
exudev issue extend 2566 "what the analysis turned up"
exudev issue remark 2566 "duplicate of #2134, check before starting"
exudev issue resolve 2566 "what was done" --author Claude-JG     #bumps the micro version
exudev issue abandon 2566 "decided against, because ..."
exudev issue triage                                #the open issues by type and effort
exudev issue serve                                 #the same in a browser, read AND write
exudev issue mode --release | --dev                              #fact 26
```

`exudev issue serve` opens a local page (`http://127.0.0.1:8099/`, standard library only) that
lists, searches and filters the issues and writes through the same functions as the verbs above
— which is the tool for a backlog pass, where a command line is not. It answers on the loopback
interface only, and **deleting an issue is not on it**: that stays a file operation with a commit
behind it.

`exudev -n issue resolve ...` prints what it would do and writes nothing — worth using before
anything that touches the version.

**The same through the API**, which the CLI calls and which a script can import:

| call | effect |
|---|---|
| `RaiseIssue(issueName, description, issueType='EXTENSION', fileName='', lineNumber='', deadline='', author='JG', priority='')` | appends a `RAISED` issue; default deadline +180 days |

| `RaiseIssueDict(issueDict)` | same, full control over fields |
| `ResolveIssue(issueNumber, notes='', author='JG')` | marks `RESOLVED`, stamps date, **bumps the micro version**; `notes` becomes `releaseNotes` and the working remarks are cleared |
| `AbandonIssue(issueNumber, reason, author='JG')` | closes without resolving; the reason is mandatory and becomes `releaseNotes` |
| `ExtendIssue(issueNumber, text, author='JG')` | appends a dated paragraph to the description of an OPEN issue and changes nothing else (R8.3.3) |
| `RemarkIssue(issueNumber, text, author='JG', replace=False)` | writes `workingRemarks` of an OPEN issue; appends by default (R8.5.3) |
| `ChangeIssue(issueNumber, key, value)` | change one field; the enum fields are checked here too |
| `ModifyDictIssue(issueDict)` | replace a whole issue (needs `number`) |
| `GetIssue(n)` / `GetIssues()` / `NumberOfIssues()` | read-only |
| `VersionString()` / `GetMajorMinorMicroVersion()` | current version |

Fields: `number, issue, author, status, description, type, priority, date raised, deadline,
date resolved, resolved author, file, line, releaseNotes, workingRemarks, effort`.

> **Two kinds of note, two fields** (revision2026 step R8.5.3). `releaseNotes` is written when
> the issue **closes** and is **published** — in `docs/generated/trackerlog.md` and, later, in
> `CHANGELOG.md`. `workingRemarks` is what the work knows meanwhile: *duplicate of #2134*,
> *marked for deprecation*, *check whether this still happens*, *part A solved, B open*. It is
> worthless once the issue closes, so `ResolveIssue` and `AbandonIssue` **clear** it, and
> `RaiseIssue` refuses a release note: there is nothing to release yet.

> **`effort` sorts the backlog**, in human working hours without AI assistance:
> `LOW` within 2, `MEDIUM` within 16, `HIGH` within 40, `HUGE` above 40. It is a classification
> and not an estimate anyone is held to; empty means not classified. `priority` is a separate
> question and may stay empty, which is what 2,405 of the 2,567 issues say — but its spelling is
> now one of `LOW`, `NORMAL`, `HIGH`, enforced wherever an issue is written.

> **Extending an issue instead of rewriting it.** When the first analysis turns up more than the
> issue says, `ExtendIssue` appends it with the date and the author. It refuses a closed issue:
> that text has already been published, so it is reopened or superseded, never rewritten.

> **Author attribution.** When Claude raises or resolves an issue on the maintainer's behalf, pass
> `author='Claude-JG'` to **both** `RaiseIssue` and `ResolveIssue` — the two fields are separate
> (`author` and `resolved author`), so both need it. `JG` alone means Johannes worked it himself.
> This keeps the tracker honest about who did what without needing a separate audit trail.

The value lists live in `issueTracker.py` (`issueStatuses`, `issueTypes`, `issuePriorities`,
`issueEfforts`) and nowhere else — this used to be a second list here and the two disagreed:

- `status`: `RAISED`, `RESOLVED`, `ABANDONED`
- `type`: `BUG, FIX, CHANGE, EXTENSION, IMPROVEMENT, TESTING, DOCU, EXAMPLE, CHECK, IDEA`
- `priority`: `''` (none), `LOW`, `NORMAL`, `HIGH`
- `effort`: `''` (not classified), `LOW`, `MEDIUM`, `HIGH`, `HUGE`

#### `BUG` vs `FIX` — the distinction is user-facing

`BUG` is not "anything broken". It is reserved for defects a *package user* has to know about,
because they explain why something silently did the wrong thing:

- a solver, formulation or model gives **wrong results**;
- something **crashes** during a simulation;
- a documented feature does not work at all, with no clear message.

`FIX` covers everything that is merely annoying to *us*: a build that does not compile, a
packaging or CI defect, documentation drift, a feature unavailable in some mode. These are
visibly failing with the cause in the message, so a user never mistakes one for correct output.

Why it matters mechanically: `ConvertToLatex`/`ConvertToHTML` select on `type == 'BUG'` to build
the open-bug list and the "resolved BUG" lines of the release notes. Anything typed `BUG` is
therefore published to users. Mistyping a build defect as `BUG` costs them attention for
something that could never have affected their results; mistyping a wrong-result defect as `FIX`
hides it.

When the cause is not yet known, prefer `BUG` — it is the safe direction.

> **An issue file can be read and even edited by hand** — that was the point of R8.5 — but go
> through `exudev issue` where a verb exists: it checks the enum fields, moves a closed issue from
> `open/` to `closed/`, and rewrites the version files, which a hand edit does not.
> `python tools/checkIssues.py` says whether the store is consistent; it runs in the commit gate.

### Known inconsistencies (raise as issues; do not fix inline)

- ~~The header documents `priority` values `NO, LOW, NORMAL, HIGH`, but `ConvertToHTML` colours
  on `high` / `med` / `low`~~ — settled in revision2026 step R8.5.3: one enum, normalized in the
  data and enforced on write.
- ~~Historical `type` values include typos and variants outside the documented set~~ — settled in
  R8.7 (#2519): 39 spellings became 10 types, checked where an issue is born.
- The batch scripts that used to live in `tools/buildAndGenerate/` carried several such
  leftovers, among them a `cd` into a directory removed years ago. They were replaced by
  `exudev` in revision2026 step R5.18 and moved out of the repository.


### What next?

Answer in this order:

1. Open `BUG`s (`status != RESOLVED and type == 'BUG'`) — currently 9.
2. Open issues by priority (`HIGH`, then `MED`, then `LOW`) — currently 9 / 5 / 4.
3. The current phase of `docs/revision/exudynRevisionPlan2026.md`.

```bash
cd tools/issueTracker
$env:USERPROFILE/Anaconda/envs/venvP312/python.exe -c "import issueTracker as it; [print(i['number'], i['priority'], i['type'], i['issue']) for i in it.GetIssues() if i['status'].strip()!='RESOLVED' and i['priority'].strip().lower()=='high']"
```

## 2. Versioning

**The micro version is derived, not written.** `GetMajorMinorMicroVersion()` counts `RESOLVED`
issues and subtracts the baseline for the current minor version, so **resolving an issue *is* the
version bump**. `ResolveIssue()` then rewrites, in one pass:

```
tools/issueTracker/issues/<the issue>.json  and issues/meta.json
tools/issueTracker/trackerlog.html          the overview (not in git)
docs/generated/trackerlog.md
src/Autogenerated/versionCpp.cpp           the version reported by exudyn at runtime
version.txt                                the single version source, at the repository root
tools/issueTracker/versionName.txt         the jazz-musician release name
```

Never edit any of those seven files by hand.

### Release mode vs development mode

`versionDev` in `tools/issueTracker/issueTracker.py` (lines 56-57) selects one of two intended
modes. Both are kept; this is not a leftover switch.

| mode | `versionDev` | version | modules built | cp313 build |
|---|---|---|---|---|
| **release** | `''` | `1.11.14` | `exudynCPP` **+** `exudynCPPfast` (both platforms) | **86 s** (2026-09-16, two modules; was 168.9 s with three) |
| **development** | `'.dev1'` | `1.11.14.dev1` | one `exudynCPP`; `exudynCPPfast` only on Python 3.13 | **58.1 s** |

Switch by editing the line and running `UpdateFiles()` from `tools/issueTracker/`. revision2026 step R8.3 turns
this into `--release` / `--dev`.

Two things follow from a switch:

- **The version string changes**, so the version line of `README.rst` picks up (or loses) the
  `.dev1` suffix when the tracker stamps it — see §4 gate 2 on ordering.
- **A `.dev1` version is not installed by a plain `pip install exudyn`** — only with `--pre` or an
  exact version. A development build therefore cannot reach users by accident.

Use development mode for ordinary work: outside Python 3.13 it builds one module instead of two
and is roughly 2× faster. Switch to release only when producing a release.

**Minor bumps (1.11 → 1.12) are manual and are the maintainer's decision.** They require editing
two constants in `issueTracker.py`: append the current total resolved count as `version12xResolved`
to the `versionResolved` list, and add the release name to `versionNames`. Claude asks first, every
time. Plan step "issueTracker CLI" folds this into the tool later.

## 0a. One-time setup per clone

```bash
git config core.hooksPath tools/hooks
python tools/setupLocalWorkspace.py
```

This activates the tracked hooks in `tools/hooks/`, currently `pre-push`, which refuses to push
anything but `master`, `release/*` and tags to the public GitHub repository. **Git hooks are not
themselves version controlled** — `.git/hooks/` never travels with a clone — so this setting is
per-clone and easy to forget. Without it there is no mechanical guard against publishing `v2-dev`.

`setupLocalWorkspace.py` creates the two **untracked** working files from their committed
templates — `exudynTemplate.sln` → `exudyn.sln` and `python/pytestTemplate.py` →
`python/pytest.py`. Open `exudyn.sln` in Visual Studio, and use `python/pytest.py` as the scratch
file for trying something out with mixed Python/C++ debugging. Both are in `.gitignore`, so an
experiment **cannot** be committed by accident; previously this relied on remembering to restore
the default `pytest.py` before committing, and a forgotten restore is invisible in review. An
existing file is never overwritten (`--force` does that deliberately).

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
origin  →  git@<internal-gitlab>:<group>/exudyn.git   internal   (v2-dev tracks origin/v2-dev)
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
| **Windows x64** | this development machine; `exudev release` (or `exudev build --complete`) weekly |
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
exudev linux            # all five, via docker + WSL; exudev -n linux prints the docker command
```

Regular CI sets `EXUDYN_NOFAST=1`, which skips the `__FAST_EXUDYN_LINALG` binary and roughly halves
build time. Ordinary test runs do not exercise that binary. **Release builds must not set it.**

### The sanitizer job

`sanitizers_linux` builds Exudyn with **AddressSanitizer and UndefinedBehaviorSanitizer** and runs
the test suite against it (revision2026 step R5.6). For a library that calls arbitrary user
callbacks from C++ and hands out references into its own storage, this is the job that turns
*"it crashed with no message"* into a file and a line.

```bash
bash tools/ci/buildSanitizers.sh python3          # also runs locally, in WSL
```

Four things about it are worth knowing before reading a red run:

- **No `setup.py` change was needed.** The flags travel in `EXUDYN_EXTRA_COMPILE_ARGS` and
  `EXUDYN_EXTRA_LINK_ARGS`, which `setup.py` already appends to every extension. `CFLAGS` does not
  work there.
- **`-O1`, not `-O3`.** That alone found #2506 on the first build, before a single sanitizer check
  ran: `RaytracingSettings::maxNThreads` had no out-of-class definition, which `-O3` hid by folding
  the constant and `-O1` turned into a module that would not load.
- **`LD_PRELOAD` carries libasan *and* the compiler's libstdc++.** Python is not instrumented, so
  the ASan runtime has to come first; and if the interpreter brings its own C++ runtime - every
  conda python does - ASan intercepts `__cxa_throw` against the wrong libstdc++ and aborts with
  `CHECK failed: ... real___cxa_throw != 0`, which looks like a finding and is only a mismatch.
- **`detect_leaks=0`.** CPython and numpy hold allocations until exit by design; memory *errors*
  are still caught, only the exit-time leak report is off.

The job is `allow_failure: true` **on purpose and temporarily**: a first sanitizer pass over 107k
lines of C++ finds things, and a job that stays red trains people to ignore it. It flips to `false`
when the findings are triaged (step R5.6.1). The run log is kept as an artifact whether the job
passes or fails - the log *is* the result.

### Which tests run when

| run | what | how |
|---|---|---|
| commit gate / pull request | test models without the slow ones and without optional packages | `runTestSuite.py --fast` (12 s) or `pytest -m "not slow and not optionalPackage"` (9 s with `-n 8`) |
| full local check | all test models and mini examples | `runTestSuite.py` (22 s) or `pytest` |
| nightly / release | models, performance tests and all examples | `exudev build --complete`, or `exudev release`, which calls the three runners |
| examples | all 171 examples, in parallel, as an API check | `exudev examples`, i.e. `runTestExamples.py` (49 s; `--serial`, `--parallel=N`, `--timeout=S`, `--exit-code`) |
| C++ unit tests | the `lest` tests in `src/Tests/` | only in a build with the `performUnitTests` switch, or the VS `Debug` configuration; then `runTestSuite.py` runs them |

The two lists behind this live in `runTestSuiteRefSol.py` as data - `SlowTests()` (measured,
above 0.6 s) and `OptionalPackageTests()` (needs ngsolve, stable-baselines3, ...) - and are read
both by `--fast` and by the pytest markers, so the two runners always skip the same models.
`pytest` additionally marks `sensitive` and `unresolvedOnLinux` from the same file.

**The models are not the slow part of a nightly run**: the whole suite is 22 seconds, while the
177 examples and the performance tests dominate. Those are addressed by revision2026 steps R5.15
and R5.16, not by this split.

### pytest

`pytest` (from the repository root, configured in `pyproject.toml`) runs
`python/testing/test_testModels.py`: one test case per test model and per mini example, each in
its own interpreter, judged against the same reference values, tolerance factors and
sensitive/unresolved lists as `runTestSuite.py` - those have one definition in
`runTestSuiteRefSol.py` and `testRunnerTools.BaseTolerance()`.

```
pytest                       all models and mini examples (137 cases, ~65 s)
pytest -k ANCF               a subset by name
pytest -n 8                  with pytest-xdist: ~11 s
pytest -k plotSensorTest -s  one model with its output
```

`pytest` and `pytest-xdist` are **dev tools**: install them yourself (`pip install pytest
pytest-xdist`); neither the package nor `runTestSuite.py` needs them. **Do not start pytest with
`python/` as working directory** - the untracked scratch file `python/pytest.py` would shadow the
pytest module.

`runTestSuite.py` stays the runner for the commit gate and the release log: it writes the release-
named log, the coverage report and the overview table, and its exit code is what CI checks.

### Running the suite in parallel

`runTestSuite.py --parallel` runs every test model in its own interpreter; `--parallel=N` sets the
number of workers (default: half the cores, at least 2, at most 8). Measured on a 32-thread
machine: **22 s serial, 11 s with 8 workers, 9 s with 16**; the limit is the interpreter start of
each model, not the models themselves. It is possible because every model writes into its own
output directory (`exudyn.config.outputDirectory`, revision2026 step R5.13).

The log reports the models in the order of the reference list, whatever order they finish in, so
log and exit code do not depend on scheduling. **The gating run stays serial by default**: models
that use multithreaded solvers or ARPACK can shift in the last digits when the machine is loaded
(measured: `NGsolveCMStest`, `objectFFRFreducedOrderTest`, `superElementRigidJointTest`, all far
inside their tolerance). Use `--parallel` while developing, and the serial run for a commit gate.

### Reproducible vs sensitive tests

**All three runners take `--exit-code`** and return non-zero when something fails - CI depends on
it, and without it CI cannot fail at all. `runTestExamples.py` and `runPerformanceTests.py` got
theirs in revision2026 step R5.18.1 (#2504); until then they always returned 0 and a caller had
to read the summary out of the log. The `exudev` driver passes the flag always, with no way to
turn it off.

Two of the three mean *nothing NEW broke* rather than *everything passed*. For the examples the
exclusions are `testRunnerTools.KnownExampleFailures()` - five today, three of them missing an
optional package (#2507) - and an entry that starts passing is reported as a **dead exclusion**
by name, so the list shrinks instead of rotting. For the test suite it is the two sets below.
`runPerformanceTests.py` excludes nothing: every performance test passes today.

Not every test is reproducible across machines:

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

### The third list: known platform differences

`UnresolvedOnLinux()` holds tests that fail on Linux against the Windows reference values for a
**reproducible** reason that has not been found yet — currently eight contact and friction models,
measured 2026-09-10. They are excluded from the exit code **on Linux only**; on Windows they must
still pass, since that is where the reference values come from. Marked `L` in the overview table,
against `*` for sensitive.

The distinction matters: sensitive tests are non-deterministic and can never be pinned down;
these have a cause and are scheduled for investigation in revision plan **phase R10, revision2026 step R10.1**. The
list should shrink, and every entry removed is a real fix — treat it as a debt register, not an
exemption.

`sphereTriangleTest.py` is in that list but is not like the others: reference 3.8226, Linux
59370.97. Four orders of magnitude is a divergence, not an accuracy difference.

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
`Debug|x64` or `Release|x64` from `exudyn.sln` (created by `tools/setupLocalWorkspace.py`), or
`exudev build` (add `--fast` for the `exudynCPPfast` module, which is opt-in).

**Stale binaries: nothing to do by hand any more.** A header change rebuilds because the
`Extension` lists every header in `depends=` (#2427), and a change of compiler OPTIONS is caught by
`build/exudynBuildFlags.txt`, which makes `setup.py` delete the objects and the linked modules
before compiling (#2468). Both were once manual steps, and the manual advice — *delete
`build/temp.*`* — was **wrong for a flag change**: `build/lib.*` keeps the previously linked `.pyd`
and the wheel is assembled from that. If you ever doubt a measurement, delete the whole `build/`
directory; that is always sufficient. Note that `pip wheel` hides the build output which would tell
you what happened, unless you pass `-v` or the build fails.

### 2. Regeneration is clean — and runs *after* `ResolveIssue`

> **Order matters.** `ResolveIssue` bumps the micro version and rewrites `version.txt`, the
> version line of `README.rst` and `docs/generated/trackerlog.md`; the wheel in your environment
> then holds the previous version, which the stub gate refuses to compare against. Resolve, then
> regenerate, then build.
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

If your change added an `import` of a third-party package anywhere in `exudyn/`, `TestModels/` or
`Examples/`, also run:

```bash
python tools/checkExtras.py --check
```

It compares the `[project.optional-dependencies]` extras in `main/pyproject.toml` against the
imports actually present in the code and fails if something is installed by no extra — so
`pip install exudyn[tests]` and `pip install exudyn[all]` cannot quietly stop being sufficient.
A fresh environment is set up with those extras rather than a hand-written package list; see
[../howTo/condaEnvironments.md](../howTo/condaEnvironments.md).

If your change touched any file under `python/exudyn/`, run the linter:

```bash
python tools/checkPython.py --check
```

It runs **ruff** with the rule set fixed in `pyproject.toml` — `F` (pyflakes: undefined names,
names defined twice, unused imports) and `E4`/`E7`/`E9` (import placement, `== None`, bare
`except:`, a file that does not parse), and nothing about formatting — and compares the result
against `tools/ci/ruffBaseline.txt`. The findings that existed when the check was introduced are
tolerated; a **new** one fails. The baseline is meant to shrink: after fixing findings, regenerate
it with `python tools/checkPython.py --write`, and the tool tells you when that is due.
`--all` lists every finding by rule, ignoring the baseline. ruff is in the `lint` dependency group
(`pip install --group lint`); the check fails rather than passes if it is not installed.

If your change touched the **stub files** or anything that generates them (`definitions/`, the
emitters), also run:

```bash
python tools/checkPython.py --stubs --check
```

It runs **mypy's `stubtest`**, which imports the module and compares it name by name and signature
by signature against `__init__.pyi` and `symbolic.pyi`. Two allowlists: `tools/ci/stubtestNoise.txt`
is curated (the dunders pybind11 adds to every class, the typing helpers that exist only in the
stub) and `tools/ci/stubtestBaseline.txt` is the generated backlog, regenerated with
`--stubs --write` and meant to shrink.

> **This check reads the INSTALLED package**, as do `runTestSuite.py`, `pytest` and
> `runTestExamples.py`: they all `import exudyn` from site-packages, never from `python/exudyn/`.
> A change under `python/exudyn/` therefore proves nothing until the package is installed — a
> passing gate on an uninstalled change is a gate that tested the previous version.

If you **added, removed or renamed a `.cpp` file**, do it in `main/obj/cppsrc.vcxproj` — the
Visual Studio project is the source of truth for the compile list — and then regenerate the list
`setup.py` actually builds from:

```bash
python tools/gen_sources.py          # rewrites main/sources.json
python tools/gen_sources.py --check  # CI mode: fails on any disagreement
```

It compares the vcxproj, `main/sources.json` and the files on disk **case-exactly**, because
`src/tests/X.cpp` and `src/Tests/X.cpp` are the same file on Windows and two different ones on
Linux. Commit `sources.json` with the change; an sdist without it cannot build. The `minimal`
list in that file is *not* derived — `--minimal` also defines `EXUDYN_MINIMAL_COMPILATION`, so it
still has to be kept in sync with the C++ `#ifdef`s by hand.

Regenerate with `python tools/regenerate.py` (add `--check` to fail on Tier 1 drift). It validates
`definitions/`, runs every generator and emitter in the required order from any directory, and
reports Tier 1 (plan §4.2) and Tier 2 (plan §4.3) differences. The order lives in one place, its
`generatorScripts` list — do not run the scripts by hand: revision2026 step R4.3 is moving outputs from the old
generators to separate emitters in `tools/generators/` (all item outputs already are).
`makeAllBinariesScripts.py` is not part of it; it writes only a volatile build date.

Two measured caveats (2026-09-09, plan §3 facts 11 and 13):

- `main/src/Autogenerated/pybind_manual_classes.h` is rewritten unconditionally by
  `autoGeneratePyBindings.py` (now `tools/generators/pybindEmitter.py`), so its `// AUTO:  last modified` line **always** shows as modified.
  Ignore that one line; treat any other change in it as real drift.
- The committed documentation at `e44aca1` is **already** out of sync with
  `main/pythonDev/Examples/` — a fresh regeneration changes 50 doc files and adds 6. Until that is
  reconciled, compare against a fresh regeneration, not against the committed state.

### 3. The full test suite passes

```bash
cd main/pythonDev/TestModels
$env:USERPROFILE/Anaconda/envs/venvP312/python.exe runTestSuite.py -quiet
```

About 22 s serially — far shorter than the compile — so it runs **in full, before every commit,
never as a subset**. Record the failed-test count and compare it against the run before the change.

Two things that will otherwise look like breakage:

- **Pin scipy to 1.15.2.** scipy 1.18.0 slows the suite from ~22 s to over 10 minutes, apparently
  in the eigensolver path (revision2026 fact 19). If a run suddenly takes minutes, check the scipy version
  before looking for a regression in Exudyn.
- **The global tolerance is 5e-14 and some models sit close to it.** A failure just above it — for
  example `movingGroundRobotTest.py` at `5.0688e-14`, with result and reference agreeing to ~13
  significant digits — is floating-point noise from a different numpy/BLAS build, not a real
  break (revision2026 fact 20). Compare the reported `RESULT` and `refsol` before treating it as one.
  Per-model tolerances are revision2026 step R5.1.

**Committed logs are protected — the runner diverts rather than overwriting.** All three runners
(`runTestSuite.py`, `runTestExamples.py`, `runPerformanceTests.py`) write a release-named log into a
tracked directory, truncating it at startup *before any test runs*. Since 2026-09-10 they check
first:

- target does not exist → written normally (the release flow: the version was just bumped)
- target exists → the log is **diverted to `python/logs/tmp/`** (gitignored), the tests run
  as usual, and a message names the override
- `--overwrite-log` → the existing log is replaced deliberately

Existence is the test, so this also covers the multi-machine case: the committed log from another
machine with the same platform and Python is present, and is not clobbered.

`python/logs/tmp/` is one shared directory for all three runners, so clearing it is a single
delete. The release-named logs go to `python/logs/{testmodels,examples,performance}/`.

### What is in a log

The header records what results depend on: Exudyn version and build date, CPU and core count, and
the installed versions of the relevant packages (`numpy`, `scipy`, `matplotlib`, `ngsolve`, …,
listed as `not installed` when absent). scipy 1.18 vs 1.15 already changed suite runtime by more
than an order of magnitude, so this is not decoration.

The log ends with a per-test overview — one fixed-width line per test with result, error, effective
tolerance and runtime, for both TestModels and MiniExamples, with sensitive tests marked `*`. That
table is how `SensitiveTests()` gets populated: diff two of them from different machines and the
non-reproducible tests stand out.

**Which compiled module is under test.** A release ships two: `exudynCPP`, built for the baseline
instruction set, and `exudynCPPfast`, without range checks and with AVX2. By default every runner
uses the first. To run against the second:

```bash
python runTestSuite.py --fast-module          # also works with --parallel
python runPerformanceTests.py --fast-module
EXUDYN_MODULE=fast python -m pytest -q -n 8   # pytest has no flag; the variable is the mechanism
```

`--fast-module` sets `EXUDYN_MODULE=fast`, which is read by `exudyn/__init__.py` *before* the C++
module is imported. It is an environment variable rather than a flag precisely because **child
processes inherit it**: `--parallel` and `pytest -n` run each model in its own interpreter. An
explicit `sys.exudynFast` in a script still wins over it. Note `--fast` is a different option
entirely — the pull-request subset.

Two things follow automatically. The suite applies `AVX2ReferenceSolutionUpdate()` (a second set of
reference values for the models whose results move under AVX2) and says so in the log; and the log
file gets a `_fast` suffix, so the two runs do not overwrite each other. If the fast module cannot
be loaded — no AVX2 on this CPU, or it is not in the installed package — the run **stops** rather
than quietly testing the regular module under a fast-module log name.

### Release testing matrix

Decided 2026-09-16, adjusted to the two variants of step R2.10:

| what | against which Python versions |
|---|---|
| full `runTestSuite.py`, default module | every supported version |
| full `runTestSuite.py --fast-module` | the **oldest** and the **second newest** (today 3.10 and 3.13) |
| examples | one version, default module |
| `runPerformanceTests.py` | `--fast-module`, plus one default-module run to compare against |

The newest version is deliberately not the fast-mode target: right after a release its packages are
the unstable part, so a failure there would almost never be about the module. **Keep every log** —
the `_fast` suffix is what makes that possible.

**Performance logs are per machine.** Set `EXUDYN_MACHINE_ID` once per machine (e.g. `i7-1370P`)
and `runPerformanceTests.py` files its logs in that subfolder, since timings from a laptop and a
workstation are not comparable. Without it, a legacy fallback still routes any 20-core machine to
`i7-1370P/`; that fallback goes away once the variable is set everywhere.

`Examples` are **not** run here: they take many minutes and only check that scripts do not crash
(they time out after a few seconds each, and are not compared for identical results). Run them
with `exudev examples` for releases and large steps only.

### 4. Docs and plan are updated

- `docs/manual/*.md`, wherever user-visible behaviour changed; the generated pages follow from
  `definitions/` and the docstrings.
- The step status in `docs/revision/exudynRevisionPlan2026.md`. If a fact in info document §3 turned out to
  be wrong, correct it there rather than working around it.

## 5. Committing

After the gates pass:

1. `ResolveIssue(issueNumber, notes='...')` — this bumps the micro version and rewrites the
   version files listed in §2. **Then re-run the generators and build the wheel**, because the
   documentation and the installed module carry the version string — see gate 2.
2. Present the maintainer with an overview: files changed, gate results (build, drift, test count),
   the issue resolved, and the proposed commit message.
3. **Wait for explicit approval.** Claude commits only on a clear go-ahead.
4. **Claude never pushes, to any remote, ever.** revision2026 step R1.5 adds a `pre-push` hook refusing
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

One driver, `exudev` (`exudev.bat` in the repository root; `python tools/exudev` elsewhere). It
replaced the sixteen batch files of `tools/buildAndGenerate/` in revision2026 step R5.18. Every
command has a `--help`, quiet is the default and `-v/--verbose` turns the tools' output back on;
**`exudev -n <command>` prints the command lines it would run and runs nothing**, which is the
quickest way to see how a step actually works. The commands are listed in
[`tools/exudev/README.md`](../../tools/exudev/README.md): `generate`, `build`, `test`, `examples`,
`perf`, `docs`, `linux`, `release`, `clean`, `env`.

Two points that catch people out. **`--fast` is opt-in**: `pyproject.toml` has
`compileExudynFast = true`, so a plain `pip wheel .` builds `exudynCPPfast` while `exudev build`
does not unless asked. And **`exudev env`** is the first thing to run when a test fails in one
environment only - it prints python, exudyn and numpy per environment, which is how the numpy
dependence of issues #2501 and #2502 was found.
