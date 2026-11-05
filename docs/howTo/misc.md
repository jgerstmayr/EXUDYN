# Misc

Notes for whoever builds, debugs or presents Exudyn: the traps of the build, what MSVC accepts and GCC does not, the
components of Visual Studio 2022, converting videos with ffmpeg, and matplotlib recipes for results.

| note | what it is for |
|---|---|
| [buildQuirks.md](buildQuirks.md) | MSVC↔GCC flags, `/bigobj`, ABI tags, debugging a startup crash |
| [gccVsMsvcTraps.md](gccVsMsvcTraps.md) | what MSVC accepts and GCC does not, and why |
| [visualStudio2022.md](visualStudio2022.md) | the Visual Studio 2022 components and `dumpbin /version` |
| [convertVideosFfmpeg.md](convertVideosFfmpeg.md) | videos and GIFs from the images Exudyn writes |
| [matplotlibExamples.md](matplotlibExamples.md) | plotting recipes for results files |

```{toctree}
:hidden:
:maxdepth: 2

/docs/howTo/buildQuirks
/docs/howTo/gccVsMsvcTraps
/docs/howTo/visualStudio2022
/docs/howTo/convertVideosFfmpeg
/docs/howTo/matplotlibExamples
```
