#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  A local web page over the issue store (revision2026 step R8.5.1), started with
#           "exudev issue serve". Reading 270 open issues, searching them and writing one is
#           done here instead of in a command line, because that is what a backlog pass is:
#           look, decide, write, look again.
#
#           WHY A WEB PAGE AND NOT A GUI: a Qt front-end would cost PySide6 as a dependency,
#           against rule 6 of CLAUDE.md, and would not work over SSH. This is http.server and
#           one HTML page from the standard library, which is already installed everywhere.
#
#           WHY IT WRITES THROUGH issueTracker: the tracker checks the enum fields, moves a
#           closed issue from open/ to closed/, and rewrites version.txt and the tracker pages.
#           A server that wrote JSON files itself would be a second, worse tracker. Every button
#           of the page ends in the same function a script would call.
#
#           WHAT IT DOES NOT DO: delete an issue. That should be rare - a wrongly raised one -
#           and deliberate, so it stays a file operation with a commit behind it.
#
#           ONLY ON THE LOOPBACK INTERFACE: the store is the version of the package and the
#           server has no authentication, so it binds 127.0.0.1 and refuses a request whose
#           Host header names anything else (a browser on another machine cannot reach it, and
#           a page in this browser cannot use a rebound name to reach it either).
#
# Usage:    exudev issue serve [--port 8099] [--no-browser] [--author NAME]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-21 (revision2026 step R8.5.1)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json
import os
import sys
import threading

#the tracker is the API; it lives beside this file and is not a package
if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import issueStore                                                             #noqa: E402
import issueTracker                                                           #noqa: E402

#how many issues one listing sends at most; the whole store is 2,568 issues and nobody reads them
#in one page, while a search that finds 300 has to say so rather than pretend there were 200
listLimit = 400


#%%******************************************************************************************************
#THE REQUEST LAYER. It is deliberately free of sockets: HandleRequest takes a method, a path, a
#query dictionary and a payload, and returns (status, contentType, body). That is what makes the
#page testable at all - the tests call it directly against a copy of the store, with no port and
#no browser (python/testing/test_issueTracker.py).
def JsonResponse(data, status=200):
    return (status, 'application/json; charset=utf-8',
            json.dumps(data, indent=1).encode('utf-8'))


def ErrorResponse(message, status=400):
    return JsonResponse({'ok': False, 'error': str(message)}, status)


def IssueSummary(issue):
    """one issue as the list shows it; the description is not sent 400 times"""
    return {name: issue[name] for name in
            ['number', 'title', 'status', 'type', 'effort', 'priority', 'dateRaised']}


def MatchingIssues(query):
    """the issues a filter bar asks for: status, the three enum fields, and a text search over
    the title, the description and the remarks"""
    issues = list(reversed(issueTracker.GetIssues()))

    status = (query.get('status') or 'open').lower()
    if status == 'open':
        issues = [issue for issue in issues if issue['status'] not in issueStore.closedStatuses]
    elif status == 'closed':
        issues = [issue for issue in issues if issue['status'] in issueStore.closedStatuses]

    for name in ['type', 'effort', 'priority']:
        wanted = (query.get(name) or '').strip().upper()
        if wanted:
            issues = [issue for issue in issues if issue[name].strip().upper() == wanted]

    search = (query.get('search') or '').strip().lower()
    if search:
        def Matches(issue):
            if search.lstrip('#').isdigit() and str(issue['number']) == search.lstrip('#'):
                return True
            return any(search in str(issue[name]).lower()
                       for name in ['title', 'description', 'workingRemarks', 'releaseNotes'])
        issues = [issue for issue in issues if Matches(issue)]

    return issues


def Meta():
    """what the page needs to build its selects and its header: the enums come from the tracker
    so that the page cannot offer a value the tracker would refuse"""
    return {'version': issueTracker.VersionString(),
            'types': issueTracker.issueTypes,
            'efforts': issueTracker.issueEfforts,
            'priorities': issueTracker.issuePriorities,
            'statuses': issueTracker.issueStatuses,
            'fields': issueStore.issueFields,
            'total': issueTracker.NumberOfIssues(),
            'closed': issueStore.ClosedCount()}


def ExistingIssue(payload):
    """the issue a POST names, or a message saying that there is none"""
    try:
        number = int(payload.get('number'))
    except (TypeError, ValueError):
        raise ValueError('no issue number in the request')
    issue = issueStore.Load(number)
    if issue is None:
        raise ValueError('there is no issue ' + str(number))
    return number


def HandleGet(path, query):
    if path in ['/', '/index.html']:
        return (200, 'text/html; charset=utf-8', pageHtml.encode('utf-8'))

    if path == '/api/meta':
        return JsonResponse(Meta())

    if path == '/api/issues':
        issues = MatchingIssues(query)
        return JsonResponse({'issues': [IssueSummary(issue) for issue in issues[:listLimit]],
                             'matching': len(issues), 'limit': listLimit})

    if path == '/api/issue':
        try:
            issue = issueStore.Load(int(query.get('number')))
        except (TypeError, ValueError):
            return ErrorResponse('no issue number in the request')
        if issue is None:
            return ErrorResponse('there is no issue ' + str(query.get('number')), 404)
        return JsonResponse(issue)

    return ErrorResponse('unknown path "' + path + '"', 404)


def HandlePost(path, payload):
    """every writing path calls the function a script would call - see the header of this file"""
    author = (payload.get('author') or 'JG').strip() or 'JG'

    if path == '/api/raise':
        number = issueTracker.RaiseIssue(payload.get('title', ''), payload.get('description', ''),
                                         issueType=payload.get('type', '').strip().upper(),
                                         fileName=payload.get('file', ''),
                                         lineNumber=payload.get('line', ''),
                                         author=author,
                                         priority=payload.get('priority', ''))
        if payload.get('effort', '').strip():
            issueTracker.ChangeIssue(number, 'effort', payload['effort'])
        return JsonResponse({'ok': True, 'number': number, 'issue': issueStore.Load(number)})

    number = ExistingIssue(payload)

    if path == '/api/extend':
        issueTracker.ExtendIssue(number, payload.get('text', ''), author=author)
    elif path == '/api/remark':
        issueTracker.RemarkIssue(number, payload.get('text', ''), author=author,
                                 replace=bool(payload.get('replace')))
    elif path == '/api/resolve':
        issueTracker.ResolveIssue(number, notes=payload.get('notes', ''), author=author)
    elif path in ['/api/close', '/api/abandon']:
        issueTracker.CloseIssue(number, reason=payload.get('reason', ''), author=author)
    elif path == '/api/modify':
        field = payload.get('field', '')
        if field not in issueStore.issueFields:
            raise ValueError('there is no field "' + str(field) + '" in an issue')
        #the tracker refuses the fields it owns, and refuses a CLOSED issue unless force is
        #passed; the page asks the person before it sends force, and shows the refusal otherwise
        issueTracker.ChangeIssue(number, field, payload.get('value', ''),
                                 force=bool(payload.get('force')))
    else:
        return ErrorResponse('unknown path "' + path + '"', 404)

    return JsonResponse({'ok': True, 'number': number, 'issue': issueStore.Load(number)})


def HandleRequest(method, path, query=None, payload=None):
    """the whole server, without a socket. ValueError is what the tracker raises when something
    is not allowed - an unknown effort, a closed issue that someone extends - and its message is
    written for a person, so it is shown as it is."""
    try:
        if method == 'GET':
            return HandleGet(path, query or {})
        if method == 'POST':
            return HandlePost(path, payload or {})
        return ErrorResponse('method ' + str(method) + ' is not supported', 405)
    except ValueError as error:
        return ErrorResponse(error)


#%%******************************************************************************************************
#THE SOCKET AROUND IT
def AllowedHost(header):
    """only this machine talks to this server. The check is on the Host HEADER and not only on the
    bound address, because a name that resolves to 127.0.0.1 lets a page in this browser reach a
    server that is bound to the loopback interface."""
    name = (header or '').split(':')[0].strip().strip('[]').lower()
    return name in ['', 'localhost', '127.0.0.1', '::1']


#ONE WRITER AT A TIME. The server is threaded (see Serve), because a browser holds sockets open
#that carry no request; the store must still see one change at a time.
requestLock = threading.Lock()


def RequestHandler():
    """the handler class, built here so that importing this module costs no http.server import"""
    import http.server
    import urllib.parse

    class Handler(http.server.BaseHTTPRequestHandler):
        #HTTP/1.1 so that a browser may keep a connection open between two fetches; every
        #response of this server carries a Content-Length, which is what that requires
        protocol_version = 'HTTP/1.1'
        timeout = 30                    #an idle connection is closed instead of held forever

        #one line per request would bury the output of the tracker functions, which is what the
        #maintainer wants to see in the terminal
        def log_message(self, format, *args):                                 #noqa: A002
            pass

        def Respond(self, response):
            (status, contentType, body) = response
            self.send_response(status)
            self.send_header('Content-Type', contentType)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def Refuse(self):
            self.Respond(ErrorResponse('this server answers only on this machine', 403))

        def do_GET(self):                                                     #noqa: N802
            if not AllowedHost(self.headers.get('Host')):
                return self.Refuse()
            parts = urllib.parse.urlparse(self.path)
            query = {name: values[0] for (name, values)
                     in urllib.parse.parse_qs(parts.query).items()}
            with requestLock:
                response = HandleRequest('GET', parts.path, query=query)
            self.Respond(response)

        def do_POST(self):                                                    #noqa: N802
            if not AllowedHost(self.headers.get('Host')):
                return self.Refuse()
            length = int(self.headers.get('Content-Length') or 0)
            try:
                payload = json.loads(self.rfile.read(length).decode('utf-8') or '{}')
            except ValueError as error:
                return self.Respond(ErrorResponse('the request is not JSON: ' + str(error)))
            parts = urllib.parse.urlparse(self.path)
            with requestLock:
                response = HandleRequest('POST', parts.path, payload=payload)
            self.Respond(response)

    return Handler


def MakeServer(port):
    """the socket, on the loopback interface and THREADED (#2571). It is a function of its own so
    that the test can start the very server Serve() starts, rather than one it configures itself -
    the bug was precisely in this configuration."""
    import http.server

    server = http.server.ThreadingHTTPServer(('127.0.0.1', port), RequestHandler())
    server.daemon_threads = True
    return server


def Serve(port=8099, openBrowser=True, author='JG'):
    """run until Ctrl+C.

    THREADED, and that is not a preference (#2571): a browser opens speculative connections that
    carry no request, and a single-threaded server accepts one of them and then blocks in
    readline() until the browser closes it again. The page itself had already been answered, so
    it rendered - and every fetch behind it waited, with no error anywhere. The store still sees
    one change at a time, because requestLock is held across each request."""
    server = MakeServer(port)
    address = 'http://127.0.0.1:' + str(port) + '/'
    print('issue tracker on ' + address + '   (' + str(issueTracker.NumberOfIssues())
          + ' issues, version ' + issueTracker.VersionString() + ', author ' + author + ')')
    print('Ctrl+C to stop; issues are written through issueTracker.py, so the version files and '
          'the tracker pages follow every resolve')

    if openBrowser:
        import webbrowser
        webbrowser.open(address + '#author=' + author)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('')
        print('issue tracker stopped')
    finally:
        server.server_close()
    return 0


#%%******************************************************************************************************
#THE PAGE. One file, inline, no framework and no request to anything outside this server: it has
#to work on a machine with no network, and a maintainer has to be able to read it.
#
#IT IS A RAW STRING, and that is the point, not a detail (#2574): a "\n" in this text belongs to
#the JavaScript. In an ordinary Python string Python eats it and writes a REAL newline into the
#middle of a JavaScript string literal, which is a syntax error, which kills the whole script -
#including the error handlers meant to report such things. The page then renders its frame and
#stays empty, in silence, which is exactly what the maintainer saw.
pageHtml = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Exudyn issues</title>
<style>
 :root { color-scheme: light dark; }
 body { font-family: system-ui, "Segoe UI", sans-serif; font-size: 14px; margin: 0;
        display: flex; flex-direction: column; height: 100vh; }
 header { padding: 8px 12px; border-bottom: 1px solid #8884; display: flex; gap: 8px;
          align-items: center; flex-wrap: wrap; }
 header b { font-size: 15px; margin-right: 4px; }
 #panes { display: flex; flex: 1; min-height: 0; }
 #listPane { width: 46%; overflow: auto; border-right: 1px solid #8884; }
 #detailPane { flex: 1; overflow: auto; padding: 12px 16px; }
 table { border-collapse: collapse; width: 100%; }
 td { padding: 3px 6px; border-bottom: 1px solid #8882; vertical-align: top; }
 tr.issue:hover { background: #8882; cursor: pointer; }
 tr.selected { background: #4a90d922; }
 .nr { font-family: ui-monospace, Consolas, monospace; white-space: nowrap; }
 .tag { font-size: 11px; padding: 1px 5px; border-radius: 3px; background: #8883;
        white-space: nowrap; }
 .RESOLVED { background: #3a9a3a44; } .CLOSED { background: #99999944; }
 .HUGE, .HIGH { background: #d9534f44; } .MEDIUM { background: #f0ad4e44; }
 .LOW { background: #5bc0de44; }
 h2 { margin: 0 0 4px 0; font-size: 17px; }
 .field { margin: 10px 0; }
 .label { font-size: 12px; opacity: 0.7; }
 pre.text { white-space: pre-wrap; margin: 2px 0; font-family: inherit; }
 textarea { width: 100%; min-height: 52px; font: inherit; box-sizing: border-box; }
 input[type=text] { font: inherit; }
 button { font: inherit; padding: 3px 10px; }
 #message { padding: 6px 12px; background: #d9534f33; display: none; white-space: pre-wrap; }
 .row { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; margin: 6px 0; }
 .danger { border: 1px solid #d9534f88; }
</style>
</head>
<body>
<header>
 <b>Exudyn issues</b>
 <select id="status">
  <option value="open">open</option>
  <option value="closed">closed</option>
  <option value="all">all</option>
 </select>
 <select id="type"></select>
 <select id="effort"></select>
 <select id="priority"></select>
 <input type="text" id="search" placeholder="search or #number" size="22">
 <button id="newIssue">new issue</button>
 <span style="flex:1"></span>
 <span class="label">author</span><input type="text" id="author" size="10" value="JG">
 <span class="label" id="versionLabel"></span>
</header>
<div id="message"></div>
<div id="panes">
 <div id="listPane"><table><tbody id="list"></tbody></table><div id="count" class="label"
      style="padding:8px 12px"></div></div>
 <div id="detailPane"></div>
</div>
<script>
let meta = null, issues = [], current = null;

function El(tag, attributes, children) {
    const node = document.createElement(tag);
    for (const name in (attributes || {})) {
        if (name === 'text') node.textContent = attributes[name];
        else if (name.startsWith('on')) node.addEventListener(name.slice(2), attributes[name]);
        else node.setAttribute(name, attributes[name]);
    }
    for (const child of (children || [])) if (child) node.appendChild(child);
    return node;
}

function Message(text) {
    const box = document.getElementById('message');
    box.textContent = text || '';
    box.style.display = text ? 'block' : 'none';
}

//a failed request must SAY so: this page once rendered its frame and stayed empty, because the
//server was blocked and nothing ever answered (#2571). Silence is the one unacceptable outcome.
async function Get(path) {
    let response;
    try { response = await fetch(path); }
    catch (error) { Message('no answer from the server (' + path + '): ' + error); return null; }
    const text = await response.text();
    let data = null;
    try { data = JSON.parse(text); }
    catch (error) {
        Message('the server did not answer with JSON (' + response.status + '): '
                + text.slice(0, 400));
        return null;
    }
    if (!response.ok) { Message(data.error || 'request failed'); return null; }
    Message('');
    return data;
}

async function Post(path, payload) {
    payload.author = document.getElementById('author').value.trim() || 'JG';
    const response = await fetch(path, {method: 'POST', headers: {'Content-Type':
        'application/json'}, body: JSON.stringify(payload)});
    const data = await response.json();
    if (!response.ok) { Message(data.error || 'request failed'); return null; }
    Message('');
    await LoadList();
    await LoadMeta();
    return data;
}

function FillSelect(id, values, label) {
    const select = document.getElementById(id);
    select.appendChild(El('option', {value: '', text: label}));
    for (const name in values) select.appendChild(El('option', {value: name, text: name}));
    select.addEventListener('change', LoadList);
}

async function LoadMeta() {
    const data = await Get('/api/meta');
    if (!data) return;
    const first = meta === null;
    meta = data;
    document.getElementById('versionLabel').textContent =
        'version ' + meta.version + ' - ' + meta.closed + ' closed of ' + meta.total;
    if (first) {
        FillSelect('type', meta.types, 'any type');
        FillSelect('effort', meta.efforts, 'any effort');
        FillSelect('priority', meta.priorities, 'any priority');
    }
}

function Query() {
    const value = id => document.getElementById(id).value;
    return '/api/issues?status=' + value('status') + '&type=' + value('type')
         + '&effort=' + value('effort') + '&priority=' + value('priority')
         + '&search=' + encodeURIComponent(value('search'));
}

async function LoadList() {
    const data = await Get(Query());
    if (!data) return;
    issues = data.issues;
    const list = document.getElementById('list');
    list.textContent = '';
    for (const issue of issues) {
        const row = El('tr', {class: 'issue' + (current && current.number === issue.number ?
                                                ' selected' : ''),
                              onclick: () => Show(issue.number)}, [
            El('td', {class: 'nr', text: '#' + issue.number}),
            El('td', {}, [El('span', {class: 'tag ' + issue.status, text: issue.status})]),
            El('td', {}, [El('span', {class: 'tag', text: issue.type})]),
            El('td', {}, [issue.effort ? El('span', {class: 'tag ' + issue.effort,
                                                     text: issue.effort}) : null]),
            El('td', {text: issue.title})]);
        list.appendChild(row);
    }
    document.getElementById('count').textContent = data.matching
        ? data.matching + ' matching issues'
            + (data.matching > data.issues.length ? ', ' + data.issues.length + ' shown' : '')
        : 'no issue matches these filters';
}

function EnumField(issue, field, values, closed) {
    const select = El('select', {onchange: async event => {
        //a closed issue has been published - its text stands in the release notes of a released
        //version - so changing one is a deliberate act and is asked for (maintainer 2026-09-21)
        if (closed && !confirm('#' + issue.number + ' is ' + issue.status + ' since '
                + issue.dateResolved + ' and is PUBLISHED in the release notes.\n\n'
                + 'Change its ' + field + ' to "' + event.target.value + '" anyway?')) {
            Show(issue.number);
            return;
        }
        const data = await Post('/api/modify', {number: issue.number, field: field,
                                                value: event.target.value, force: closed});
        if (data) Show(issue.number);
    }});
    select.appendChild(El('option', {value: '', text: '(none)'}));
    for (const name in values) {
        const option = El('option', {value: name, text: name + ' - ' + values[name]});
        if ((issue[field] || '') === name) option.selected = true;
        select.appendChild(option);
    }
    return select;
}

function TextBlock(label, text) {
    if (!text || !text.trim()) return null;
    return El('div', {class: 'field'}, [El('div', {class: 'label', text: label}),
                                        El('pre', {class: 'text', text: text})]);
}

function WriteBox(label, buttonText, send, extra) {
    const area = El('textarea', {placeholder: label});
    const children = [El('button', {text: buttonText, onclick: () => send(area.value)})];
    if (extra) children.push(extra);
    return El('div', {class: 'field'}, [El('div', {class: 'label', text: label}), area,
                                        El('div', {class: 'row'}, children)]);
}

async function Show(number) {
    const issue = await Get('/api/issue?number=' + number);
    if (!issue) return;
    current = issue;
    const closed = issue.status !== 'RAISED';
    const pane = document.getElementById('detailPane');
    pane.textContent = '';
    pane.appendChild(El('h2', {text: '#' + issue.number + '  ' + issue.title}));
    pane.appendChild(El('div', {class: 'label', text:
        issue.status + ', ' + issue.type + ', raised ' + issue.dateRaised + ' by '
        + (issue.author || '?')
        + (closed ? ', closed ' + issue.dateResolved + ' by ' + issue.resolvedAuthor : '')
        + (issue.file ? ', ' + issue.file + (issue.line ? ':' + issue.line : '') : '')}));

    pane.appendChild(El('div', {class: closed ? 'row danger' : 'row'}, [
        El('span', {class: 'label', text: closed ? 'CLOSED AND PUBLISHED - effort' : 'effort'}),
        EnumField(issue, 'effort', meta.efforts, closed),
        El('span', {class: 'label', text: 'priority'}),
        EnumField(issue, 'priority', meta.priorities, closed),
        El('span', {class: 'label', text: 'type'}),
        EnumField(issue, 'type', meta.types, closed)]));

    pane.appendChild(TextBlock('description', issue.description));
    pane.appendChild(TextBlock('working remarks (cleared when it closes)', issue.workingRemarks));
    pane.appendChild(TextBlock('release notes (published)', issue.releaseNotes));

    if (closed) return;

    pane.appendChild(WriteBox('extend the description', 'extend',
        text => Post('/api/extend', {number: issue.number, text: text})
                    .then(data => data && Show(issue.number))));

    const replace = El('input', {type: 'checkbox', id: 'replaceRemarks'});
    pane.appendChild(WriteBox('working remarks', 'remark',
        text => Post('/api/remark', {number: issue.number, text: text,
                                     replace: replace.checked})
                    .then(data => data && Show(issue.number)),
        El('label', {}, [replace, document.createTextNode(' replace')])));

    pane.appendChild(WriteBox('release note - resolving BUMPS THE VERSION', 'resolve',
        text => confirm('resolve #' + issue.number + '? This bumps the micro version.')
            && Post('/api/resolve', {number: issue.number, notes: text})
                   .then(data => data && Show(issue.number))));

    //CLOSED covers everything except RESOLVED - obsolete, won't fix, duplicate, superseded,
    //not reproducible, abandoned - and the reason says which (D13)
    pane.appendChild(WriteBox('reason - closing without resolving also counts for the version',
        'close',
        text => confirm('close #' + issue.number + ' without resolving it?')
            && Post('/api/close', {number: issue.number, reason: text})
                   .then(data => data && Show(issue.number))));
}

function NewIssue() {
    current = null;
    const pane = document.getElementById('detailPane');
    pane.textContent = '';
    pane.appendChild(El('h2', {text: 'a new issue'}));

    const title = El('input', {type: 'text', size: '70', placeholder: 'the issue in one line'});
    const description = El('textarea', {placeholder: 'what it is about, in prose'});
    const type = El('select', {});
    for (const name in meta.types)
        type.appendChild(El('option', {value: name, text: name + ' - ' + meta.types[name]}));
    const effort = El('select', {});
    effort.appendChild(El('option', {value: '', text: 'effort: not classified'}));
    for (const name in meta.efforts)
        effort.appendChild(El('option', {value: name, text: name + ' - ' + meta.efforts[name]}));
    const priority = El('select', {});
    priority.appendChild(El('option', {value: '', text: 'priority: none'}));
    for (const name in meta.priorities)
        priority.appendChild(El('option', {value: name, text: name}));
    const file = El('input', {type: 'text', size: '40', placeholder: 'file (optional)'});
    const line = El('input', {type: 'text', size: '6', placeholder: 'line'});

    pane.appendChild(El('div', {class: 'field'}, [El('div', {class: 'label', text: 'title'}),
                                                  title]));
    pane.appendChild(El('div', {class: 'row'}, [type, effort, priority, file, line]));
    pane.appendChild(El('div', {class: 'field'}, [El('div', {class: 'label',
                                                             text: 'description'}), description]));
    pane.appendChild(El('div', {class: 'row'}, [El('button', {text: 'raise', onclick: async () => {
        const data = await Post('/api/raise', {title: title.value, description: description.value,
            type: type.value, effort: effort.value, priority: priority.value,
            file: file.value, line: line.value});
        if (data) Show(data.number);
    }})]));
}

window.addEventListener('error', event => Message('the page has a problem: '
    + event.message + '  (' + event.filename + ':' + event.lineno + ')'));
window.addEventListener('unhandledrejection',
    event => Message('the page has a problem: ' + event.reason));

document.getElementById('newIssue').addEventListener('click', NewIssue);
document.getElementById('search').addEventListener('input', LoadList);
document.getElementById('status').addEventListener('change', LoadList);
if (location.hash.startsWith('#author='))
    document.getElementById('author').value = decodeURIComponent(location.hash.slice(8));

LoadMeta().then(LoadList);
</script>
</body>
</html>
"""


#%%******************************************************************************************************
if __name__ == '__main__':
    #"exudev issue serve" is the way in; this is here so that the file can be run on its own
    sys.exit(Serve())
