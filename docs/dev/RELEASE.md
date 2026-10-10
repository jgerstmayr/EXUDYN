(sec-dev-release)=
# Releasing Exudyn

How a version goes from `v2-dev` to GitHub, PyPI and the documentation: the branches and what starts what,
the commands in order, a rehearsal on GitHub before anything is published, and the traps. Versioning and the
two modes are in [WORKFLOW.md](WORKFLOW.md) §3, the remotes in §4.

## What starts what

| what | where | started by |
|---|---|---|
| development | branch `v2-dev`, on the internal GitLab (`origin`) | every day |
| releases | branch `master`, on GitHub (`github`) **and** GitLab (`origin`) | a release only; always a fast-forward to the release commit |
| release tag | `vX.Y.Z` on the release commit, on both remotes | the maintainer, **after** GitHub CI is green |
| GitHub CI `wheels.yml` | 24 jobs: Windows, Linux x86-64, Linux ARM, macOS (universal2), Python 3.10-3.15 each; cibuildwheel builds and runs the test suite with `--exit-code`; the artifact `merged-wheels` holds all wheels | a push to `master`, a pull request, or *Run workflow* - **not** a tag |
| GitHub Pages `documentation.yaml` | the HTML documentation on jgerstmayr.github.io | a push to `master`, or *Run workflow* |
| Read the Docs | `latest` from `master` | a push to `master`; other branches and tags only as configured in the Read the Docs project (*Versions*, *Automation rules*) |
| GitLab CI `.gitlab-ci.yml` | Linux x86-64 wheels and tests | a tag, the schedule, or *Run pipeline* - **not** a branch push |
| PyPI | the wheels, uploaded with twine | the maintainer |

The micro version counts resolved issues, so every fix made during a release moves the version. A tag that
was never pushed is deleted and made again; `master` only ever moves forward, so no step needs a force push.

**GitHub Actions** must be allowed to run the actions of the workflows: *Settings → Actions → General*,
either all actions, or the actions created by GitHub plus `JamesIves/github-pages-deploy-action@*`. An
allow-list of the account's own actions only blocks `actions/checkout`, and every job fails at once.

## The release, step by step

From the repository root, in PowerShell with `venvExuP313` active. Only the lines with `git push` and the
upload touch a remote; each push to GitHub needs the 2FA.

### 1. Prepare
1. All issues of the release resolved, the working tree clean, `exudev build --complete` green.
2. The release version - a decision of the maintainer ([CLAUDE.md](https://github.com/jgerstmayr/EXUDYN/blob/master/CLAUDE.md), *Ask before*):
   ```powershell
   exudev issue mode --release
   exudev issue bump --minor
   ```
   Regenerate, the four gates, commit.
3. **Rehearse on GitHub** from that commit, see {ref}`below <rehearse-on-github>`. It runs the 24 wheel jobs before
   anything is tagged or published.

### 2. Build and tag
4. The release notes, cut from `CHANGELOG.md`, and the pdf of the documentation:
   ```powershell
   exudev docs --pdf
   python -c "import sys; sys.path[:0]=['tools/exudev','tools']; import commands; commands.WriteReleaseNotes()"
   ```
   `exudev release --tag` does this too, together with the local build of every Windows wheel, both test
   suites per version, the performance tests, the examples and the Linux wheels in Docker. It is the local
   check of a release; the published wheels are the ones of GitHub CI, which builds and tests all four
   platforms from the tagged commit.
5. The tag, with the release notes as its message:
   ```powershell
   git tag -a vX.Y.Z -F dist/RELEASE_NOTES.md
   git show vX.Y.Z --no-patch
   ```
   If `exudev release --tag` stops with "the working tree is dirty" because the build rewrote
   `python/exudyn/__init__.pyi` and `symbolic.pyi` (only their line endings; #2915), restore them with
   `git checkout -- python/exudyn/__init__.pyi python/exudyn/symbolic.pyi` and tag by hand.

### 3. Push master, without the tag
6. Move `master` to the release commit. This is a local push: it refuses anything but a fast-forward and
   leaves the working tree alone, which `git checkout master` would not:
   ```powershell
   git push . "vX.Y.Z^{commit}:master"
   git rev-parse master
   ```
7. ```powershell
   git push github master             # starts wheels.yml and GitHub Pages; Read the Docs builds latest
   git push origin master
   git push origin v2-dev
   ```

### 4. Wait for green, collect the wheels
8. All 24 jobs of `wheels` green. If one fails: its log and the test log (artifact `uploaddata-tests-*`) to
   `tmp/GitHub`, fix on `v2-dev`, resolve the issue, `git tag -d vX.Y.Z` (it was never pushed), and steps 4-7
   again with the new version.
9. Download `merged-wheels` and take the wheels of exactly this version:
   ```powershell
   Remove-Item distPyPI\*.whl -ErrorAction SilentlyContinue
   Copy-Item <unzipped>\merged-artifacts\exudyn-X.Y.Z-*.whl distPyPI\
   dir distPyPI                       # 24 wheels: win_amd64, manylinux x86_64, manylinux aarch64, macosx
   ```
   `distPyPI/` is ignored by git. One source per platform; a local wheel from `dist/` is uploadable only in
   its `manylinux` and `win_amd64` forms - PyPI refuses `linux_x86_64`.

### 5. Tag, PyPI, GitHub release
10. ```powershell
    git push github vX.Y.Z
    git push origin vX.Y.Z            # starts the GitLab tag pipeline
    twine check distPyPI/*.whl
    twine upload --skip-existing distPyPI/*.whl
    ```
    twine needs version 6.1 or newer (and pkginfo 1.12 or newer): the wheels carry Metadata-Version 2.4
    (`License-Expression`, `License-File`), which an older twine reports as "unrecognized or malformed". The
    dependency group `release` of `pyproject.toml` has both: `pip install --group release` (pip 25.1 or newer).
    It reads the API token from `%USERPROFILE%\.pypirc` (`username = __token__`, `password = pypi-...`), or
    from `--config-file <path>`; `--verbose` shows which file and which URL it uses.
11. *Releases → Draft a new release* on GitHub, from tag `vX.Y.Z`: the body is `dist/RELEASE_NOTES.md`, the
    attachment `dist/exudynDocumentationVX.Y.Z.pdf`.
12. Check pypi.org/project/exudyn (description, 20 files), `pip install -U "exudyn[basic]"` and
    `python -m exudyn info` in a fresh environment, the GitHub start page, GitHub Pages and Read the Docs.

### 6. Back to development
13. ```powershell
    exudev issue mode --dev
    ```
    Regenerate, the gates, commit on `v2-dev`. The logs of the release go in with it
    (`python/logs/*/*X.Y.Z*`); the pre-commit hook refuses a personal path in them, written as `%USERPROFILE%`
    instead.
14. The logs of the development versions leave `python/logs` for the maintainer's archive; old wheels leave
    `dist/`.

(rehearse-on-github)=
## Rehearse on GitHub

A branch on GitHub runs the wheels workflow by hand, on the exact commit, and publishes nothing:
```powershell
git push github v2-dev:release/X.Y          # the pre-push hook lets release/* through to GitHub
```
*Actions → wheels → Run workflow*, branch `release/X.Y`, with *build as a release* checked. Fix and push again until
all 24 jobs are green, then
```powershell
git push github --delete release/X.Y
```
- A branch push starts nothing on GitHub; *Run workflow* is the only trigger.
- Start **only `wheels`** there. *Deploy GitHub Pages*, started by hand on that branch, would publish its
  documentation.
- Read the Docs builds `master` only, unless the branch is activated as a version in the Read the Docs project.
- *Build as a release* sets `EXUDYN_RELEASE_BUILD=1`, and `setup.py` then builds `exudynCPPfast` for every Python
  version, as for a release version; a development version otherwise builds it for Python 3.13 only (WORKFLOW.md §3,
  #2922). So a rehearsal can run from any commit, also before `exudev issue mode --release`; run from the commit
  after it (step 2), its wheels are the release wheels if that commit becomes the release commit.
- The same workflow runs **weekly on `master`** (Monday, 03:17 UTC), the last release: a red run there is a change on
  GitHub's side - the runner image, a Python patch release, a package - and shows before the next release does.
- The metadata of the wheels, which PyPI shows as the project description, can be read in the rehearsal
  artifacts before anything is uploaded.

A rehearsal costs one manual run and no version number. Everything it finds would otherwise be found after
`master` is pushed, at the price of a fix, a new version and a new tag.

## Traps

| trap | what to do |
|---|---|
| GitHub shows `README.md` before `README.rst` | there is no `README.md` in the repository root; `README.rst` is the landing page of GitHub and Read the Docs |
| the PyPI description is the inline text of `[project.readme]` in `pyproject.toml`, part of every wheel | check it, and its links, before the wheels are built - a wrong text needs a new version |
| `exudynCPPfast` exists only on x86-64 Windows and Linux (no macOS, no Linux ARM) | its compiler options are x86-64 ones; setup.py decides where it is built |
| the test suite in CI runs with `--exit-code`; the reference values are the Windows ones | known platform differences are in `UnresolvedOnLinux()` and `UnresolvedOnMacOS()`, the latter also for Linux ARM ([WORKFLOW.md](WORKFLOW.md) §5) |
| the Python of the cibuildwheel tests on Windows has no tkinter | a test that needs the dialogs skips them where tkinter is missing |
| the fast-module suite has a reference set of its own (`AVX2ReferenceSolutionUpdate()`) | renew it whenever the regular values move ([WORKFLOW.md](WORKFLOW.md) §8, *What is in a log*); the weekly `exudev build --complete --fast` and `exudev test --fast` show it |
| `exudev linux` runs in the local copy of the manylinux image, which docker does not renew | after a new Python version (cp315, #2928) `docker pull quay.io/pypa/manylinux_2_28_x86_64` once; `/opt/python` of the image lists the versions it has |
| a large file pushed once stays in the history (GitHub warns above 50 MB, refuses above 100 MB) | build output stays out of the repository (`.gitignore`) |
