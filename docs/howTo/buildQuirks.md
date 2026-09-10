# Build quirks and traps

Non-obvious things that cost real time to discover, salvaged from how-to files that were otherwise
retired in the 2026-09 cleanup. Nothing here is a walkthrough — those are better answered by the
official documentation or by asking an AI. What is here is the opposite: specifics that no general
source will tell you, because they are particular to this project or to a combination of tools.

Each entry names where it came from, so the full original can be found in git history if needed.

## Compiler and build flags

**MSVC to GCC flag translation.** When porting a build option, the two families differ:

```
/std:c++17   ->  -std=c++17
/openmp      ->  -fopenmp
```

*(from `ubuntuPythonSetup.txt`)*

**Force a specific compiler through setuptools** by setting the environment before the build runs;
setuptools honours `CC`/`CXX`:

```python
os.environ["CC"]  = "gcc-8"
os.environ["CXX"] = "gcc-8"
```

*(from `ubuntuPythonSetup.txt`)*

**`/bigobj` is required on MSVC** — "add `/bigobj` to additional compiler options, as pybind11
generates many sections". Without it, larger translation units fail to compile once the pybind11
binding code grows.

*(from `Python_spyder.txt`)*

## Debugging

**A crash on startup hides its own cause.** Running under the debugger and typing `continue` makes
Python print the real traceback instead of dying silently:

```bash
python3 -m pdb myFirstExample.py
# if it crashes immediately, type:
continue
# ==> continue will write further information
```

*(from `ubuntuPythonSetup.txt`)*

**Decode the ABI tag to see which Python a binary was built for.** The tag is in the file name:

```
exudynCPP.cp311-win_amd64.pyd   ->  cp311 = Python 3.11
```

An "unable to import" that mentions a module which is plainly present is usually this: the `.pyd`
was built for a different interpreter.

*(from `pybind howto.txt`; `VS2022howto.txt` has the `dumpbin /version` equivalent)*

## Third-party interactions

**Anaconda 2020.02 breaks the sparse solvers in Eigen.** Recorded verbatim because the symptom
appears far from the cause: *"THE FOLLOWING WORKS, BUT MAKES PROBLEMS with sparse solvers in
EIGEN: Install Anaconda 2020.02"*. If sparse solves misbehave after an environment change, suspect
the distribution before the code.

*(from `VS2019python37_64bit.txt`)*

**scipy version affects eigenvalue solver performance substantially.** Measured 2026-09-10: with an
unpinned scipy, `abaqusImportTest.py` alone took 60 s of a 106 s test suite; pinning `scipy==1.15.2`
returned it to normal. This is not only a test-suite concern — it applies to any Exudyn code using
sparse eigenvalue solves. See revision plan fact 19/19a.

**Intel MKL is not used.** Earlier notes described wiring MKL into the Visual Studio project behind
a `USE_EXUDYN_MKL` define. That define no longer exists anywhere in the source, so the integration
is gone; do not go looking for it.

*(from `intelMKL.txt`, verified absent 2026-09-10)*

## Windows conveniences

**Stop Windows sleeping during a long simulation:**

```python
import ctypes
# prevent sleep / monitor off
ctypes.windll.kernel32.SetThreadExecutionState(0x80000002)
# back to normal
ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
```

*(from `pythonHowTo.txt`)*
