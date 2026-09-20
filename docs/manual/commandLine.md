(sec-commandline)=
# The command line: `python -m exudyn`

Some things are wanted from a shell rather than from a script: *what did I actually install?*,
*show me that sensor file*, *is this installation working at all?*. The installed package answers
them directly:

```
python -m exudyn <command> [options]
```

There are four commands. Each one parses its own options, so `python -m exudyn <command> --help`
is the authoritative list — this page describes what the commands are *for*, not every flag they
take.

| command | what it does |
|---|---|
| `monitor` | live view of a results file while it is being written, see {ref}`sec-resultsmonitor` |
| `plot` | one static plot of sensor or solution files, then exit |
| `info` | version, location and environment of this installation |
| `demo` | run a built-in demo model |

`python -m exudyn --version` prints the version string alone, which is convenient in scripts.

## `info` — what to put in a bug report

```
python -m exudyn info
```

prints the Exudyn version and the build it comes from, the path of the installed package, which
compiled module is loaded (`EXUDYN_MODULE`), the output directory, the Python version and platform,
and which of numpy, scipy, matplotlib, networkx, NGsolve and pytest are installed.

That block answers most of the questions a bug report otherwise needs a conversation about, so
**paste it into an issue** rather than describing the installation in prose.

## `plot` — a figure without writing a script

```
python -m exudyn plot solution/sensorPosition.txt
python -m exudyn plot s0.txt s1.txt --columns 0,1 --save position.png
```

Without `--columns`, every column except time is drawn over time. `--save` writes the figure
(`.png`, `.pdf`, `.svg`) instead of, or in addition to, showing it; a run that only saves needs no
window and works over SSH.

## `demo` — does this installation work?

```
python -m exudyn demo          #a simple model with the renderer
python -m exudyn demo 2
```

The two demos of `exudyn.demos`. If these run, the compiled module, numpy, matplotlib and the
renderer are in order — which is the first thing to establish when something else does not work.

## `monitor` — the results monitor

```
python -m exudyn monitor --last
```

attaches to the most recently written results file. The monitor has a page of its own:
{ref}`sec-resultsmonitor`.

## Why this is not simply `exudyn ...`

A console script would put the name `exudyn` on the `PATH` of every environment the package is
installed into. That is a promise about a name, and the set of commands is young, so the
installation deliberately makes no such promise yet: `python -m exudyn` costs a few characters more
and can change without breaking anybody's shell.

The commands live in a plain dictionary inside `exudyn/__main__.py`, and each one imports what it
needs only when it is called — an unused command costs nothing at startup, and the table is the
place where a plugin can add its own command later.
