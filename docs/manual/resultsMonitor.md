(sec-resultsmonitor)=
# The results monitor

A long simulation, a parameter variation that runs overnight, a genetic optimization on a cluster:
all of them write their results to a file *while* they run. The results monitor shows that file as
it grows — it re-reads what has been appended since the last update and redraws the curves, until
the window is closed.

It reads the four kinds of file Exudyn writes: a **sensor** file, a **coordinates solution** file,
and the `resultsFile` of **`ParameterVariation`** and **`GeneticOptimization`**. Which one it is
does not have to be said; the header of the file tells it.

## Three ways to call it

```
python -m exudyn monitor --last                 #the newest results file that is found
python -m exudyn monitor solution/genetic.txt   #a given file
python -m exudyn.misc.resultsMonitor f.txt      #the module directly, same options
```

and from a script or from the Spyder console:

```python
from exudyn.misc.resultsMonitor import MonitorResults
MonitorResults('solution/genetic.txt', logY=True, updatePeriod=0.5)
```

`MonitorResults(...)` is the function; the command line is a thin layer over it. Everything the
options do is available as an argument, and `help(MonitorResults)` describes them.

`MonitorResults(...)` **blocks** until its window is closed, which is what is wanted in a console
and not what is wanted in a script that still has to run the simulation. For that there is

```python
from exudyn.misc.resultsMonitor import StartResultsMonitor
StartResultsMonitor('solution/sensorPos.txt', updatePeriod=0.5)
mbs.SolveDynamic(simulationSettings)     #the plot follows the file while this runs
```

`StartResultsMonitor(...)` starts the monitor as a **second process** - `python -m exudyn monitor`
with the options it was given - and returns at once. The two processes share nothing but the file,
which is the protocol they already had, so there is no plotting inside the solver and no question of
threads. The file does not have to exist yet: the monitor waits for the first row.

The process is **not** stopped when the script ends, so the plot is still there when a short
simulation is over; the returned `subprocess.Popen` is the handle for a script that wants it gone.
Nothing is started when windows are suppressed
(`EXUDYN_SUPPRESS_UI_WINDOW_OPEN`, `exudyn.special.userInterface.suppressPlots`), so a test that runs such a
script neither opens a window nor leaves a process behind. `springDamperTutorial.py` watches a sensor
file this way and `3SpringsDistance.py` the coordinates solution.

## Finding the file

Typing a path is the part that used to make the monitor awkward, so there are three ways around
it:

- **`--last`** takes the most recently written results file it finds.
- **no file name at all** opens a file dialog; without tkinter, a numbered list in the terminal.
- **`--list-files`** prints the results files that were found and their type, and exits.

It looks in `exudyn.config.outputDirectory`, in `solution/` and in the current directory;
`--dir DIR` adds another place and may be given repeatedly.

## Choosing what is drawn

By default a sensor or solution file is drawn over time, with every column except time; an
optimization file is drawn over its varied parameters.

```
python -m exudyn monitor f.txt --list-columns       #which column is which, with indices
python -m exudyn monitor f.txt -y 2,3 --log-y       #only these columns, logarithmic
```

`--list-columns` exists because guessing a column index from a file is the second thing that used
to make the monitor awkward. For a parameter variation, `--color-variations` draws the first
parameter only and gives every variation of the remaining parameters its own colour.

## The control panel

Next to the plot there is a small window (tkinter, part of the standard library — no extra
dependency) which changes what you are looking at **without restarting the run**:

- pause and resume the updates, and change the update period;
- logarithmic x / y, autoscale on or off;
- one checkbox per curve, up to 16 curves;
- *save figure…* and *save data…* (the plotted data as CSV);
- *open file…*, which points the monitor at another results file.

`--no-panel` leaves it out. If tkinter is missing, or windows are suppressed, the monitor says so
in one line and draws the plot without the panel.

## Settings that survive

What you set in the panel is stored in `~/.exudyn/resultsMonitor.json` — update period, log scales,
autoscale, window size, whether the panel is shown, and the directory last used. The next monitor
starts the way the last one ended. `--no-settings` neither reads nor writes that file, which is what
a reproducible script or a test wants.

An option given on the command line always wins over the stored setting.

## In a script, in a test, on a server

```
python -m exudyn monitor f.txt --once --save results.png
```

`--once` draws the file as it is, writes `--save` if given, and returns instead of entering the
update loop. That is what makes the monitor usable where nobody can close a window — a CI job, a
batch script, a machine reached over SSH — and it is how the monitor itself is tested
(`python/TestModels/resultsMonitorTest.py`).

The same happens automatically when window opening is suppressed
(`EXUDYN_SUPPRESS_UI_WINDOW_OPEN`, `exudyn.config.suppressPlots`): the monitor draws once and
returns rather than waiting for a human who is not there.

## Every option

```
python -m exudyn monitor --help
```

The options are grouped there — what to plot, appearance, updating — and that list is generated
from the program itself, so it cannot go out of date the way a list printed in a manual can.
