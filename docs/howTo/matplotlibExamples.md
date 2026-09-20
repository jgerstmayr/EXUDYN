# Matplotlib notes for Exudyn results

Recipes that come up again and again when plotting Exudyn results by hand. For sensor and solution
files there is usually no need to write any of this:
`exudyn.plot.PlotSensor(...)` does it, and `python -m exudyn plot file.txt` does it from a shell.
These notes are for the cases where you want the figure exactly your way.

## Spyder: inline or a real window

```python
%matplotlib auto      #plots in their own window
%matplotlib inline    #plots inside the console
```

The same switch is in *Preferences → IPython console → Graphics → Backend*: *Inline* draws into the
console, *Automatic* opens a window. A window is what you want for anything you will zoom into.

## Reading a results file

Exudyn writes comma-separated columns with `#` comment lines, which is exactly what `np.loadtxt`
expects:

```python
import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('solution/coordinatesSolution.txt', comments='#', delimiter=',')
plt.plot(data[:, 0], data[:, 3], 'b-')      #column 3 over time (column 0)
plt.show()
```

`--list-columns` of the results monitor prints which column is which:
`python -m exudyn monitor solution/coordinatesSolution.txt --list-columns`.

## A plot that is worth putting in a paper

```python
x = np.linspace(0, 2, 100)
plt.plot(x, x, 'b-', linewidth=0.5, label='linear')   #1.0 is too thick for a PDF; 0.5-0.7 is right
plt.title('interactive test')
plt.xlabel('time (s)')
plt.ylabel('displacement (m)')
plt.grid(True, 'major', 'both')
plt.legend(loc='upper right')
plt.tight_layout()                                    #keeps the labels inside the figure
plt.savefig('figure.pdf')                             #.png and .svg work the same way
plt.show()
```

`loc` takes `'upper left'`, `'lower right'`, `'center'`, and so on — plus `'best'`, which finds the
spot with the least overlap and is slow on large data sets.

## Colour, marker and line style in one string

```python
plt.plot([1, 2, 3, 4, 5], [1, 2, 3, 4, 10], 'go-')    #green dots, solid line
```

- colours: `b` blue, `g` green, `r` red, `c` cyan, `m` magenta, `y` yellow, `k` black, `w` white
- markers: `.` `o` (point, circle), `v ^ < >` (triangles), `s 8 * P + x D d`
- lines: `-` `--` `-.` `:`

So `'r*--'` is red stars on a dashed line, `'ks.'` black squares on a dotted one, `'bD-.'` blue
diamonds on a dash-dot line.

## Axes: scale, limits, grid, ticks

```python
ax = plt.axes(xscale='log', yscale='log')     #or 'linear'
ax = plt.gca()                                #the axes of the current figure

ax.grid(True, 'major', 'both')                #{'major','minor','both'}, {'both','x','y'}

plt.xlim(xMin, xMax)
plt.ylim(ax.yaxis.get_data_interval())        #exactly the range of the data
ax.autoscale(True)                            #ax.margins(0., 0.05) leaves a little air

import matplotlib.ticker as ticker
ax.yaxis.set_major_locator(ticker.MaxNLocator(8))     #at most 8 ticks
ax.minorticks_on()

plt.rcParams.update({'font.size': 12})        #scales every font of the figure
```

## Two figures at once

```python
(figure1, axes1) = plt.subplots()
(figure2, axes2) = plt.subplots()

axes1.plot(data[:, 1], data[:, 2], 'b-', label='case 1')   #phase plot
axes1.set_title('Phase plot')
axes1.set_xlabel('displacement (u)')
axes1.set_ylabel('velocity (v)')
axes1.grid(True, 'major', 'both')
axes1.legend()

axes2.plot(data[:, 0], data[:, 1], 'b-', label='case 1')   #over time
axes2.set_xlabel('time (t)')
axes2.set_ylabel('displacement (u)')
axes2.legend()

figure1.tight_layout()
figure2.tight_layout()
figure1.savefig('phasePlot.pdf')
figure2.savefig('displacement.pdf')

plt.close('all')          #and this is how you get rid of them again
```

## A marker on every point, without hiding the line

Three plots on top of each other: the line, a white disc, the marker. The white disc is what keeps
the line from showing through the marker.

```python
plt.plot(data[:, 0], data[:, 1], color='blue', linestyle='-', linewidth=2)
plt.plot(data[:, 0], data[:, 1], color='white', marker='o', linestyle='none', markersize=14)
plt.plot(data[:, 0], data[:, 1], color='blue', marker='o', linestyle='none', markersize=10)

from matplotlib.lines import Line2D
legendElements = [Line2D([0], [0.1], color='b', marker='o', markersize=10, lw=2, label='Line')]
plt.legend(handles=legendElements)
plt.show()
```

## LaTeX in the labels

```python
plt.rcParams.update({'text.usetex': True,
                     'font.family': 'serif',
                     'font.serif': ['Palatino']})
plt.ylabel(r'\bf{phase field} $\phi$', {'color': 'C0', 'fontsize': 20})
plt.xticks((-1, 0, 1), ('$-1$', r'$\pm 0$', '$+1$'), color='k', size=20)
plt.text(-1, .30, r'gamma: $\gamma$', {'color': 'r', 'fontsize': 20})
```

This needs a LaTeX installation; see matplotlib's
[usetex demo](https://matplotlib.org/stable/gallery/text_labels_and_annotations/usetex_demo.html).
