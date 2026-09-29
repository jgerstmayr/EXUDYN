#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The issues over time, as one matplotlib figure (#2752): on the left axis the total, the
#           closed and the open issues; on the right axis the bugs and fixes, total and open. An
#           issue counts as raised on dateRaised and as closed on dateResolved; the few resolved
#           issues of the early years that carry no dateResolved count as closed on dateRaised.
#
# Usage:    exudev issue plot                 #opens the figure
#           exudev issue plot --save FILE     #writes it to FILE (png, pdf, svg) instead
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import datetime


def _Date(text):
    """'2026-09-29' or '2026-09-29 21:27' as a date; None if empty"""
    text = str(text).strip()
    if text == '':
        return None
    return datetime.datetime.strptime(text[:10], '%Y-%m-%d').date()


def Counts(issues):
    """per day on which anything changed: the cumulative counts, as a dict of lists
    (dates, total, closed, open, bugs, openBugs, fixes, openFixes)"""
    events = {}                                     #date -> list of (type, +1 raised / -1 closed)
    for issue in issues:
        raised = _Date(issue['dateRaised'])
        if raised is None:
            continue
        events.setdefault(raised, []).append((issue['type'], 1))
        if issue['status'] != 'RAISED':
            closed = _Date(issue['dateResolved']) or raised
            events.setdefault(closed, []).append((issue['type'], -1))

    counts = dict((key, []) for key in ['dates', 'total', 'closed', 'open', 'bugs', 'openBugs', 'fixes', 'openFixes'])
    state = dict((key, 0) for key in counts if key != 'dates')
    for date in sorted(events):
        for (issueType, change) in events[date]:
            if change > 0:
                state['total'] += 1
                state['bugs'] += issueType == 'BUG'
                state['fixes'] += issueType == 'FIX'
            else:
                state['closed'] += 1
            state['openBugs'] += change*(issueType == 'BUG')
            state['openFixes'] += change*(issueType == 'FIX')
        state['open'] = state['total'] - state['closed']
        counts['dates'].append(date)
        for key in state:
            counts[key].append(state[key])
    return counts


def Plot(issues, fileName=None):
    """the figure; shown, or written to fileName"""
    import matplotlib
    if fileName:
        matplotlib.use('Agg')                       #no window when the figure only goes to a file
    import matplotlib.pyplot as plt

    counts = Counts(issues)
    dates = counts['dates']
    figure, left = plt.subplots(figsize=(11, 6))
    right = left.twinx()
    lines = []
    for (key, label, color) in [('total', 'total issues', 'black'), ('closed', 'closed issues', 'tab:green'),
                                ('open', 'open issues', 'tab:blue')]:
        lines += left.step(dates, counts[key], where='post', color=color, linewidth=2, label=label)
    for (key, label, color, style) in [('bugs', 'total bugs', 'tab:red', '--'), ('openBugs', 'open bugs', 'tab:red', '-'),
                                       ('fixes', 'total fixes', 'tab:orange', '--'), ('openFixes', 'open fixes', 'tab:orange', '-')]:
        lines += right.step(dates, counts[key], where='post', color=color, linestyle=style, linewidth=1.2,
                            label=label + ' (right axis)')
    left.set_xlabel('date')
    left.set_ylabel('issues')
    right.set_ylabel('bugs and fixes')
    left.grid(True, alpha=0.3)
    left.legend(lines, [line.get_label() for line in lines], loc='upper left')
    left.set_title('Exudyn issues over time: ' + str(counts['total'][-1]) + ' raised, '
                   + str(counts['open'][-1]) + ' open (' + str(counts['openBugs'][-1]) + ' bugs, '
                   + str(counts['openFixes'][-1]) + ' fixes)')
    figure.autofmt_xdate()
    figure.tight_layout()
    if fileName:
        figure.savefig(fileName)
        plt.close(figure)
    else:
        plt.show()
    return counts
