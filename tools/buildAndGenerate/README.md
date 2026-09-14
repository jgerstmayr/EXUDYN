# tools/buildAndGenerate/ — Windows build, test and documentation scripts

Batch scripts for the maintainer's Windows workflow (WSL for the linux wheels). They can be started
from any directory: every script locates the repository from its own path (`%~dp0..\..`), and conda
through `condaActivate.bat`. Nothing in here contains machine-specific paths.

**Conda:** `condaActivate.bat` finds the base installation via `EXUDYN_CONDA_ROOT` (if set), else
`CONDA_EXE` (defined by `conda init`), else `conda` on `PATH`. The environments are named
`venvP310` … `venvP314` for the version matrix and `venvExuP313` for generation and docs, see
[docs/howTo/condaEnvironments.md](../../docs/howTo/condaEnvironments.md).

| script | what it does |
|---|---|
| `runPythonScripts.bat [makeAll] [nodoc]` | regenerate everything via `tools/regenerate.py` (the generator driver `tools/generators/generate.py` plus the drift report), then build the html docs |
| `makeSphinxDoc.bat [env]` | html documentation: `sphinx-build -b html . _build -E` from the repository root |
| `makeDoc.bat` | the LaTeX documentation `docs/theDoc/theDoc.pdf` (pdflatex + bibtex8, twice) |
| `runTestSuite.bat [all\|P3xx]` | test suite, all Python versions or one |
| `runPerformanceTests.bat [all\|P3xx]` | performance tests |
| `runTestExamples.bat [all\|P3xx]` | the Examples set (slow; default P312) |
| `makeInstallBinaries.bat [noscripts] [P3xx]` | regenerate, build and install for one version (default P313) |
| `makeWindowsBinaries.bat [all\|P3xx]` | build the Windows wheels |
| `makeAndTestAllBinaries.bat [nodoc] [noscripts] [notests] [noinstall] [nolinux] [none]` | the full release path: clean, regenerate, all wheels, all tests, docs and linux wheels in separate windows |
| `makeUbuntuManyLinuxWheels.bat [nofast]` | manylinux wheels in docker via WSL, running `manylinuxBuild.sh` (same per-version code as GitLab CI, `tools/ci/buildManylinux.sh`) |
| `makeUbuntuWheels.bat [quiet] [nofast] [P3xx ...]` | linux wheels directly in WSL conda environments (not manylinux) |
| `buildInstallSingleVersion.bat [yes\|no]` | `pip wheel` and reinstall in the active environment; `yes` skips the install |
| `removeBuildsAndEggs.bat` | remove the Windows build directories and eggs |
| `execWithPythonVersion.bat`, `execWithAllPythonVersions.bat` | run a command in one or all `venvP3xx` environments (test runners from `python\TestModels`, everything else from the repository root) |
| `condaActivate.bat` | activate conda base, see above |

**Check before trusting a test run:** the `venvP3xx` environment must have the current build
installed (`import exudyn; exudyn.__version__` equal to `version.txt`); a stale install makes the
test suite fail on version-dependent results.
