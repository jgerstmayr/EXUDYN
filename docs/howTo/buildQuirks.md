# Build quirks and traps

Specifics that no general source tells, because they are particular to this project or to a combination of tools.
The build itself is in {ref}`sec-dev-build`, the environments and the scipy pin in
[condaEnvironments.md](condaEnvironments.md).

## Compiler and build flags

**MSVC to GCC flag translation.** When porting a build option, the two families differ:

```
/std:c++17   ->  -std=c++17
/openmp      ->  -fopenmp
```

**`/bigobj` is required on MSVC**, as pybind11 generates many sections. Without it, larger translation units fail
to compile once the binding code grows.

## Debugging

**A crash on startup hides its own cause.** Running under the debugger and typing `continue` makes
Python print the real traceback instead of dying silently:

```bash
python3 -m pdb myFirstExample.py
# if it crashes immediately, type:
continue
```

**The ABI tag says which Python a binary was built for.** The tag is in the file name:

```
exudynCPP.cp311-win_amd64.pyd   ->  cp311 = Python 3.11
```

An "unable to import" that mentions a module which is plainly present is usually this: the `.pyd`
was built for a different interpreter. `python -m exudyn info` prints which Python and which Exudyn run, and
`dumpbin /version` what a `.pyd` is ([visualStudio2022.md](visualStudio2022.md)).

## Windows conveniences

**Stop Windows sleeping during a long simulation:**

```python
import ctypes
# prevent sleep / monitor off
ctypes.windll.kernel32.SetThreadExecutionState(0x80000002)
# back to normal
ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
```
