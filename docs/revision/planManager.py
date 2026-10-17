#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# planManager - check, read and edit the revision plan exudynRevisionPlan2026c.md
#
# The plan is one Markdown file with four sections, each exactly once and in this order:
#   # Revision plan how to          the rules, hand-written
#   # Revision plan next steps      the recommended next steps, hand-written; checked here
#   # Revision plan open steps      GENERATED: the open groups and steps down to RGx.y
#   # Revision plan groups          GENERATED overview table, then the groups and their steps
# A group or step is a heading whose level follows from its number - a group RGx is '##', a step
# RGx.y '###', RGx.y.z '####', RGx.y.z.w '#####' - and which starts with its marker and title:
#   ### **OPEN_RG4.21** Constraints that depend on velocities or on time
# Everything up to the next heading belongs to it. The marker is in the file exactly once, so a search
# for "_RG4.21**" finds the step; text refers to a step as "RG4.21", never with the marker.
#
# Usage (from any directory):
#   python docs/revision/planManager.py check              #exit 1 if the plan is not valid
#   python docs/revision/planManager.py update             #rewrite the generated parts, then check
#   python docs/revision/planManager.py list [--open] [--depth 2] [--group RG4]
#   python docs/revision/planManager.py show RG4.21
#   python docs/revision/planManager.py add RG4 --title "..." [--body "..." | --body-file f.md]
#   python docs/revision/planManager.py set RG4.21 [--status CLOSED] [--title "..."] [--body ... | --body-file ... | --append ...]
# Every command that writes runs 'update' and the check at its end.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import datetime
import io
import json
import os
import re
import sys

here = os.path.dirname(os.path.abspath(__file__))
planPath = os.path.join(here, 'exudynRevisionPlan2026c.md')
previousPlanPath = os.path.join(here, 'exudynRevisionPlan2026b.md')   #its numbers are taken
issuesPath = os.path.join(here, '..', '..', 'tools', 'issueTracker', 'issues')

sections = ['Revision plan how to', 'Revision plan next steps', 'Revision plan open steps',
            'Revision plan groups']
generatedOpen = 'open steps'
generatedGroups = 'groups table'

headingRe = re.compile(r'^(#{1,6}) (.*)$')
stepHeadingRe = re.compile(r'^(#{2,6}) \*\*(OPEN|CLOSED)_RG(\d+(?:\.\d+)*)\*\* (\S.*)$')
markerRe = re.compile(r'(OPEN|CLOSED)_RG\d')
#a reference: RG4, RG4.21, RG4.21.3 - not part of a word, a marker or a file name
referenceRe = re.compile(r'(?<![\w_./-])RG(\d+(?:\.\d+)*)(?![\w.]*\d)')
oldPlanPrefixRe = re.compile(r'revision2026b? (step|steps|group|groups) $')


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def NumberKey(number):
    return tuple(int(x) for x in number.split('.'))


def ParentOf(number):
    return number.rsplit('.', 1)[0] if '.' in number else None


def Depth(number):
    return number.count('.') + 1


class Plan:
    """the plan file as lines, with its sections and steps found"""

    def __init__(self, path=planPath):
        self.path = path
        self.lines = io.open(path, encoding='utf-8').read().replace('\r\n', '\n').split('\n')
        self.Parse()

    def Parse(self):
        self.sectionStart = {}          #name -> line index of its '# ' heading
        self.steps = {}                 #number -> dict(number, status, title, level, start, end)
        self.order = []                 #numbers in file order
        self.errors = []
        h1 = [(i, m.group(2)) for (i, m) in ((i, headingRe.match(line)) for (i, line) in enumerate(self.lines))
              if m and m.group(1) == '#']
        for (i, name) in h1:
            if name in self.sectionStart:
                self.errors.append('line %d: the section "# %s" appears twice' % (i + 1, name))
            self.sectionStart[name] = i
        for (i, name) in h1:
            if name not in sections:
                self.errors.append('line %d: "# %s" is not one of the four sections' % (i + 1, name))
        if [name for (i, name) in h1 if name in sections] != sections:
            self.errors.append('the sections must be, once each and in this order: '
                               + ', '.join('"# ' + s + '"' for s in sections))
        groupsStart = self.sectionStart.get(sections[3], len(self.lines))

        current = None
        for i in range(len(self.lines)):
            line = self.lines[i]
            hm = headingRe.match(line)
            if hm is None:
                continue
            if current is not None:
                current['end'] = i
                current = None
            if i <= groupsStart or hm.group(1) == '#':
                continue
            sm = stepHeadingRe.match(line)
            if sm is None:
                self.errors.append('line %d: a heading in the groups section that is no group or step: %s'
                                   % (i + 1, line[:80]))
                continue
            number = sm.group(3)
            step = {'number': number, 'status': sm.group(2), 'title': sm.group(4).strip(),
                    'level': len(sm.group(1)), 'start': i, 'end': len(self.lines)}
            if number in self.steps:
                self.errors.append('line %d: RG%s appears twice' % (i + 1, number))
                continue
            self.steps[number] = step
            self.order.append(number)
            current = step

    def Body(self, number):
        step = self.steps[number]
        return '\n'.join(self.lines[step['start'] + 1:step['end']]).strip('\n')

    def Children(self, number):
        return [n for n in self.order if ParentOf(n) == number]

    def Descendants(self, number):
        return [n for n in self.order if n.startswith(number + '.')]

    def Groups(self):
        return [n for n in self.order if '.' not in n]

    def SubtreeEnd(self, number):
        """the line index after the last line of the step and all its descendants"""
        numbers = [number] + self.Descendants(number)
        return max(self.steps[n]['end'] for n in numbers)

    def Text(self):
        return '\n'.join(self.lines)

    def Write(self):
        text = '\n'.join(self.lines).rstrip('\n') + '\n'
        with io.open(self.path, 'w', encoding='utf-8', newline='\n') as file:
            file.write(text)
        self.Parse()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the numbers given before: a step of revision2026b keeps its number for ever, so a new step never reuses one
def TakenNumbers():
    taken = set()
    if os.path.isfile(previousPlanPath):
        text = io.open(previousPlanPath, encoding='utf-8').read()
        taken.update(re.findall(r'\*\*RG(\d+(?:\.\d+)*)[a-z]?\*\*', text))
        taken.update(re.findall(r'^## RG(\d+) ', text, flags=re.M))
    return taken


def IssueStates():
    """issue number -> 'open' or 'closed', from the tracker store; {} if it is not there"""
    states = {}
    for (folder, state) in [('open', 'open'), ('closed', 'closed')]:
        path = os.path.join(issuesPath, folder)
        if os.path.isdir(path):
            for name in os.listdir(path):
                if name.endswith('.json') and name[:-5].isdigit():
                    states[int(name[:-5])] = state
    archive = os.path.join(issuesPath, 'archive')
    if os.path.isdir(archive):
        for name in os.listdir(archive):
            if name.endswith('.json'):
                try:
                    data = json.load(io.open(os.path.join(archive, name), encoding='utf-8'))
                    for issue in (data if isinstance(data, list) else data.get('issues', [])):
                        states[int(issue.get('number', issue.get('issueNumber', -1)))] = 'closed'
                except (ValueError, AttributeError, TypeError):
                    pass
    return states


def DeferredIssues():
    """issue number -> deferredUntil of the open issues that wait on purpose until a later date (#3024)"""
    deferred = {}
    path = os.path.join(issuesPath, 'open')
    today = datetime.date.today().isoformat()
    if os.path.isdir(path):
        for name in os.listdir(path):
            if name.endswith('.json') and name[:-5].isdigit():
                try:
                    date = str(json.load(io.open(os.path.join(path, name), encoding='utf-8')).get('deferredUntil', ''))
                except ValueError:
                    continue
                if date > today:
                    deferred[int(name[:-5])] = date
    return deferred


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the generated parts
def Marker(name, end=False):
    return '<!-- planManager: %s %s - generated by docs/revision/planManager.py update, do not edit -->' % (
        'end' if end else 'begin', name)


def OpenStepsText(plan):
    out = []
    for group in plan.Groups():
        if plan.steps[group]['status'] != 'OPEN':
            continue
        out.append('- **RG%s** %s' % (group, plan.steps[group]['title']))
        for number in plan.Descendants(group):
            step = plan.steps[number]
            if Depth(number) == 2 and step['status'] == 'OPEN':
                openBelow = sum(1 for n in plan.Descendants(number) if plan.steps[n]['status'] == 'OPEN')
                out.append('  - RG%s %s%s' % (number, step['title'],
                                              (' (%d open sub-step%s)' % (openBelow, '' if openBelow == 1 else 's')) if openBelow else ''))
    return out


def GroupsTableText(plan):
    out = ['| group | status | open steps | title |', '|---|---|---|---|']
    for group in plan.Groups():
        openCount = sum(1 for n in plan.Descendants(group) if plan.steps[n]['status'] == 'OPEN')
        out.append('| RG%s | %s | %d | %s |' % (group, plan.steps[group]['status'], openCount,
                                               plan.steps[group]['title']))
    return out


def ReplaceGenerated(plan, sectionName, name, newLines, afterSectionHeading=True):
    """the lines between the begin and end markers of 'name'; inserted after the section heading if missing"""
    begin, end = Marker(name), Marker(name, end=True)
    if begin in plan.lines and end in plan.lines:
        i, j = plan.lines.index(begin), plan.lines.index(end)
        plan.lines[i + 1:j] = newLines
    else:
        start = plan.sectionStart[sectionName]
        plan.lines[start + 1:start + 1] = ['', begin] + newLines + [end, '']
    plan.Parse()


def GeneratedIsCurrent(plan):
    stale = []
    for (name, lines) in [(generatedOpen, OpenStepsText(plan)), (generatedGroups, GroupsTableText(plan))]:
        begin, end = Marker(name), Marker(name, end=True)
        if begin not in plan.lines or end not in plan.lines:
            stale.append(name + ' (missing)')
        elif plan.lines[plan.lines.index(begin) + 1:plan.lines.index(end)] != lines:
            stale.append(name)
    return stale


def Update(plan):
    if plan.errors:          #the sections must be found before anything is generated into them
        return
    ReplaceGenerated(plan, sections[2], generatedOpen, OpenStepsText(plan))
    ReplaceGenerated(plan, sections[3], generatedGroups, GroupsTableText(plan))
    plan.Write()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Check(plan, verbose=True):
    """the list of errors (the plan is not valid) and of warnings (worth a look)"""
    errors = list(plan.errors)
    warnings = []
    steps = plan.steps

    for number in plan.order:
        step = steps[number]
        where = 'line %d (RG%s): ' % (step['start'] + 1, number)
        if step['level'] != Depth(number) + 1:
            errors.append(where + 'a heading of level %d, its number needs %d (%s)'
                          % (step['level'], Depth(number) + 1, '#' * (Depth(number) + 1)))
        parent = ParentOf(number)
        if parent is not None and parent not in steps:
            errors.append(where + 'its parent RG%s is missing' % parent)
        if '**' in step['title'] or step['title'].endswith('.'):
            warnings.append(where + 'the title is plain text without a final period')

    #order: every step after its parent and its previous sibling, siblings ascending
    for (k, number) in enumerate(plan.order):
        if k == 0:
            continue
        previous = plan.order[k - 1]
        parent = ParentOf(number)
        if parent is None:
            ok = NumberKey(previous.split('.')[0]) < NumberKey(number)
        elif previous == parent:
            ok = True
        else:
            #the previous step is a sibling or the descendant of one, with a smaller number
            prefix = previous
            while prefix is not None and ParentOf(prefix) != parent:
                prefix = ParentOf(prefix)
            ok = prefix is not None and NumberKey(prefix) < NumberKey(number)
        if not ok:
            errors.append('line %d (RG%s): out of order after RG%s'
                          % (steps[number]['start'] + 1, number, previous))

    #status: a closed step has no open step below; a group is open exactly while a step of it is open
    for number in plan.order:
        below = [n for n in plan.Descendants(number) if steps[n]['status'] == 'OPEN']
        if steps[number]['status'] == 'CLOSED' and below:
            errors.append('RG%s is CLOSED but RG%s below it is OPEN' % (number, below[0]))
        if '.' not in number and steps[number]['status'] == 'OPEN' and not below:
            errors.append('group RG%s is OPEN without an open step - close it' % number)

    #numbers of revision2026b are not given again
    taken = TakenNumbers()
    #(a step carried over from revision2026b keeps its number; a NEW step must not take a number of it)
    newNumbers = set(plan.order) - taken
    for number in newNumbers:
        parent = ParentOf(number)
        siblingsTaken = [n for n in taken if ParentOf(n) == parent]
        if siblingsTaken and NumberKey(number) < max(NumberKey(n) for n in siblingsTaken):
            warnings.append('RG%s is new, but revision2026b numbered up to RG%s there - is it a reused gap?'
                            % (number, max(siblingsTaken, key=NumberKey)))

    #markers only in the headings, and each exactly once
    for (i, line) in enumerate(plan.lines):
        if markerRe.search(line) and not stepHeadingRe.match(line):
            errors.append('line %d: a marker OPEN_RG/CLOSED_RG outside a heading - refer to a step as RGx.y'
                          % (i + 1))

    #references: RGx.y in the text exists here, or is cited with the old plan's prefix
    for (i, line) in enumerate(plan.lines):
        if stepHeadingRe.match(line):
            line = stepHeadingRe.match(line).group(4)
        for m in referenceRe.finditer(line):
            if oldPlanPrefixRe.search(line[:m.start()]):
                continue
            if m.group(1) not in steps:
                errors.append('line %d: RG%s is no step of this plan (an old one is cited as '
                              '"revision2026b step RG%s")' % (i + 1, m.group(1), m.group(1)))

    #next steps: exist and are open
    if not plan.errors:
        start = plan.sectionStart[sections[1]]
        end = plan.sectionStart[sections[2]]
        found = 0
        for i in range(start + 1, end):
            for m in referenceRe.finditer(plan.lines[i]):
                if oldPlanPrefixRe.search(plan.lines[i][:m.start()]):
                    continue
                found += 1
                if m.group(1) in steps and steps[m.group(1)]['status'] != 'OPEN':
                    errors.append('line %d: the next steps name RG%s, which is CLOSED' % (i + 1, m.group(1)))
        if found == 0:
            warnings.append('the next steps name no step')
        stale = GeneratedIsCurrent(plan)
        if stale:
            errors.append('the generated parts are not current (' + ', '.join(stale) + '): run "update"')

    #an open step whose issues are all closed is probably done
    states = IssueStates()
    if states:
        for number in plan.order:
            if steps[number]['status'] != 'OPEN' or plan.Children(number):
                continue
            #an issue cited as the origin or the context ("from #707", "since #2869", "after #2745") is not the work
            text = re.sub(r'\b(from|From|since|after) #\d{2,4}\b', '', plan.Body(number) + ' ' + steps[number]['title'])
            issues = [int(x) for x in re.findall(r'#(\d{2,4})\b', text)]
            known = [x for x in issues if x in states]
            if known and all(states[x] == 'closed' for x in known):
                warnings.append('RG%s is OPEN, but all its issues are closed: %s'
                                % (number, ', '.join('#%d' % x for x in known)))
        for number in plan.order:
            if steps[number]['status'] == 'CLOSED' and plan.Body(number).count('\n') > 12:
                warnings.append('RG%s is CLOSED and has %d lines - the details belong in the log'
                                % (number, plan.Body(number).count('\n') + 1))
    return errors, warnings


def Report(plan):
    errors, warnings = Check(plan)
    for w in warnings:
        print('  warning: ' + w)
    for e in errors:
        print('  ERROR: ' + e)
    groups = plan.Groups()
    nOpen = sum(1 for n in plan.order if '.' in n and plan.steps[n]['status'] == 'OPEN')
    if errors:
        print('planManager: the plan is NOT valid (%d errors)' % len(errors))
        return 1
    #the issues that wait on purpose: a closed step may cite them (#3024)
    deferred = DeferredIssues()
    for number in sorted(deferred):
        steps = [n for n in plan.order if re.search(r'#%d(?!\d)' % number, plan.Body(n) + ' ' + plan.steps[n]['title'])]
        print('  deferred: #%d until %s%s' % (number, deferred[number],
                                             (' (RG' + ', RG'.join(steps) + ')') if steps else ''))
    print('planManager: OK - %d groups, %d steps, %d of them open%s'
          % (len(groups), len(plan.order) - len(groups), nOpen,
             ', %d warnings' % len(warnings) if warnings else ''))
    return 0


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#editing
def NormalizeNumber(text):
    text = text.strip()
    if text.startswith('RG'):
        text = text[2:]
    if not re.fullmatch(r'\d+(\.\d+)*', text):
        raise SystemExit('planManager: "%s" is no step number (RG4, RG4.21, ...)' % text)
    return text


def BodyFrom(args):
    if getattr(args, 'body_file', None):
        return io.open(args.body_file, encoding='utf-8').read().strip('\n')
    return (getattr(args, 'body', None) or '').strip('\n')


def Add(plan, parent, title, body='', status='OPEN'):
    """a new step below parent (a new group if parent is None), with the next free number; returns it"""
    taken = TakenNumbers() | set(plan.order)
    if parent is None:
        siblings = [n for n in taken if '.' not in n]
        number = str(max(int(n) for n in siblings) + 1 if siblings else 1)
        insertAt = len(plan.lines)
    else:
        if parent not in plan.steps:
            raise SystemExit('planManager: RG%s does not exist' % parent)
        siblings = [n for n in taken if ParentOf(n) == parent]
        number = parent + '.' + str(max(NumberKey(n)[-1] for n in siblings) + 1 if siblings else 1)
        insertAt = plan.SubtreeEnd(parent)
    level = Depth(number) + 1
    new = ['%s **%s_RG%s** %s' % ('#' * level, status, number, title.strip()), '']
    if body:
        new += body.split('\n') + ['']
    #keep one empty line before the heading
    if insertAt > 0 and plan.lines[insertAt - 1].strip() != '':
        new = [''] + new
    plan.lines[insertAt:insertAt] = new
    plan.Parse()
    #a new open step opens its parents
    if status == 'OPEN':
        p = ParentOf(number)
        while p is not None:
            if plan.steps[p]['status'] == 'CLOSED':
                SetStatus(plan, p, 'OPEN')
            p = ParentOf(p)
    return number


def SetStatus(plan, number, status):
    step = plan.steps[number]
    plan.lines[step['start']] = '%s **%s_RG%s** %s' % ('#' * step['level'], status, number, step['title'])
    plan.Parse()


def Set(plan, number, status=None, title=None, body=None, append=None):
    if number not in plan.steps:
        raise SystemExit('planManager: RG%s does not exist' % number)
    step = plan.steps[number]
    if title is not None:
        step['title'] = title.strip()
    plan.lines[step['start']] = '%s **%s_RG%s** %s' % ('#' * step['level'], status or step['status'], number,
                                                     step['title'])
    if body is not None or append is not None:
        old = plan.Body(number)
        text = body if body is not None else (old + ('\n\n' if old else '') + append.strip('\n'))
        plan.lines[step['start'] + 1:step['end']] = [''] + text.split('\n') + ['']
    plan.Parse()
    #a closed group whose last open step closed: close it too; an opened step opens its group
    group = number.split('.')[0]
    openBelow = [n for n in plan.Descendants(group) if plan.steps[n]['status'] == 'OPEN']
    if '.' in number:
        if openBelow and plan.steps[group]['status'] == 'CLOSED':
            SetStatus(plan, group, 'OPEN')
        elif not openBelow and plan.steps[group]['status'] == 'OPEN':
            SetStatus(plan, group, 'CLOSED')


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Main():
    parser = argparse.ArgumentParser(description='check, read and edit exudynRevisionPlan2026c.md')
    parser.add_argument('--plan', default=planPath, help='the plan file (default: %(default)s)')
    sub = parser.add_subparsers(dest='command')
    sub.add_parser('check', help='check the plan; exit 1 if it is not valid')
    sub.add_parser('update', help='rewrite the generated parts, then check')
    p = sub.add_parser('list', help='the groups and steps, one line each')
    p.add_argument('--open', action='store_true', help='only the open ones')
    p.add_argument('--closed', action='store_true', help='only the closed ones')
    p.add_argument('--depth', type=int, default=9, help='down to RGx.y for 2, ...')
    p.add_argument('--group', default=None, help='only this group, e.g. RG4')
    p = sub.add_parser('show', help='a step with its text and the titles of its sub-steps')
    p.add_argument('number')
    p = sub.add_parser('add', help='a new step below a parent (or a new group without one)')
    p.add_argument('parent', nargs='?', default=None)
    p.add_argument('--title', required=True)
    p.add_argument('--body', default=None)
    p.add_argument('--body-file', default=None)
    p.add_argument('--status', default='OPEN', choices=['OPEN', 'CLOSED'])
    p = sub.add_parser('set', help='change the status, the title or the text of a step')
    p.add_argument('number')
    p.add_argument('--status', default=None, choices=['OPEN', 'CLOSED'])
    p.add_argument('--title', default=None)
    p.add_argument('--body', default=None)
    p.add_argument('--body-file', default=None)
    p.add_argument('--append', default=None, help='text added at the end of the step')
    args = parser.parse_args()

    plan = Plan(args.plan)
    command = args.command or 'check'
    if command == 'check':
        return Report(plan)
    if command == 'update':
        Update(plan)
        return Report(plan)
    if command == 'list':
        group = NormalizeNumber(args.group) if args.group else None
        for number in plan.order:
            step = plan.steps[number]
            if Depth(number) > args.depth or (group and number.split('.')[0] != group):
                continue
            if (args.open and step['status'] != 'OPEN') or (args.closed and step['status'] != 'CLOSED'):
                continue
            print('%s%-8s %-6s %s' % ('  ' * (Depth(number) - 1), 'RG' + number, step['status'], step['title']))
        return 0
    if command == 'show':
        number = NormalizeNumber(args.number)
        if number not in plan.steps:
            raise SystemExit('planManager: RG%s does not exist' % number)
        step = plan.steps[number]
        print('RG%s  %s  %s' % (number, step['status'], step['title']))
        print(plan.Body(number))
        for child in plan.Descendants(number):
            print('%s- RG%s %s %s' % ('  ' * (Depth(child) - Depth(number) - 1), child,
                                       plan.steps[child]['status'], plan.steps[child]['title']))
        return 0
    if command == 'add':
        parent = NormalizeNumber(args.parent) if args.parent else None
        number = Add(plan, parent, args.title, BodyFrom(args), args.status)
        print('planManager: added RG' + number)
    elif command == 'set':
        number = NormalizeNumber(args.number)
        body = BodyFrom(args) if (args.body is not None or args.body_file) else None
        Set(plan, number, args.status, args.title, body, args.append)
        print('planManager: changed RG' + number)
    plan.Write()
    Update(plan)
    return Report(plan)


if __name__ == '__main__':
    sys.exit(Main())
