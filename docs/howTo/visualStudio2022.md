# Visual Studio 2022 for Exudyn

Visual Studio 2022 Community is the primary C++ environment of Exudyn: it is what gives **mixed
Python/native debugging**, which is the most valuable capability the project has — a breakpoint in
a `CObject*.cpp` hit from a Python model, with both call stacks visible.

The solution is `exudynTemplate.sln`: copy it to `exudyn.sln` (which is untracked, so your local
settings stay yours) and open that.

## What to install

From the Visual Studio installer, the *Community* edition is enough:

| component | why |
|---|---|
| **Desktop development with C++** (~3 GB) | the compiler and libraries; Visual Studio asks for it as soon as Exudyn is loaded |
| **Windows 10/11 SDK** | without it, over a thousand includes such as `<math.h>` and `<GL/gl.h>` are not found |
| **Python development** | the Python side of mixed debugging |

The Python that Visual Studio installs is a bare one. If you use it rather than an existing conda
environment, add what the examples need:

```powershell
pip install scipy
pip install matplotlib
```

Note that plots do not stay open in that setup — end a script with `plt.show(block=True)`.

For the environments the project actually builds and tests against, see
[condaEnvironments.md](condaEnvironments.md); virtual environments inside Visual Studio are
untested.

## Which module is this, actually?

When several builds are around, `dumpbin` answers what a `.pyd` really is:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.37.32822\bin\Hostx64\x64\dumpbin' /version exudynCPP.cp313-win_amd64.pyd
```

(the MSVC version in that path is whatever your installation has). The same question from the
Python side, which is usually the faster one:

```powershell
python -m exudyn info
```

## See also

- [buildFromSource.md](buildFromSource.md) — building the wheel without Visual Studio
- [buildQuirks.md](buildQuirks.md) — what goes wrong on Windows and why
- [gccVsMsvcTraps.md](gccVsMsvcTraps.md) — what MSVC accepts and GCC does not
