#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkDefinitions - the descriptions in definitions/ obey the rules of definitions/README.md
#
# Why this check exists (#2655): a description in definitions/ is the source of a page of the
# reference manual. It goes through one converter, tools/generators/latexToMarkdown.py, and what
# the converter does not recognise is carried through to the page as itself - no warning, no
# failure, and the page builds. This is the check that names the file and the line instead.
#
# What it checks today:
#   - every ABRV:KEY names an abbreviation that the list actually has. The abbreviations are the
#     dict in tools/generators/examplesDocsEmitter.py, which also writes
#     docs/generated/abbreviations.md, so the key and its target cannot drift apart.
#   - a description holds no LaTeX outside its mathematics. This is the rule the whole of
#     RG3.14 was for, and until it existed a macro the converter did not know reached the page.
#   - a comment is <!-- ... --> ; a '%' is a comment only inside mathematics, where the engine
#     reads it, because this converter strips a '%' before the mathematics is protected.
#   - a reference to an equation is the {eq} role: a Markdown link to one leaves the PDF as an
#     undefined reference, and only the PDF says so.
#   - a literal whose value carries a backslash or mathematics is written r'...', so that
#     Python does not read a backslash-t as a tab.
#   - every heading is written at the level of the page it is placed in, and a title that means one
#     of the recurring sections is spelled like it. The old \mysubsubsubsection said the level in
#     its name and NormalizeHeadings quietly repaired whatever did not fit, so neither was checked.
#
# Usage:
#   python tools/checkDefinitions.py            #report
#   python tools/checkDefinitions.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import ast
import difflib
import glob
import io
import os
import re
import sys


def DeclaredAbbreviations(path):
    """the keys of the abbreviations dict, read as a literal - importing the emitter would pull in
    the whole generator package for one dict"""
    tree = ast.parse(io.open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and getattr(node.targets[0], 'id', '') == 'abbreviations'):
            return set(key.value for key in node.value.keys)
    raise ValueError(path + ': no "abbreviations" dict found')


#the headings that recur - a title that means one of these is spelled like it, so that the same
#section does not appear as "Connector forces" on one page and "Connector Forces" on the next.
#A title of its own is allowed; it must only not be one of these in different clothes.
RECURRING_HEADINGS = [
    'Definition of quantities',
    'Equations of motion',
    'Connector forces',
    'Connector constraint equations',
    'Geometric relations',
    'Details',
    'Marker quantities',
    'Post Newton Step',
    'Super element output variables',
    'Additional output variables for superelement node access',
    ]

#the heading level a description is written at: an item's text sits under the item's DESCRIPTION
#heading, a structure's introduction directly under the page title
HEADING_LEVEL = {'sectionText': 2}
DEFAULT_HEADING_LEVEL = 4

#keywords whose value is Python, not prose - published as a code block, so a '#' is a comment
CODE_KEYWORDS = set(['code', 'miniExample', 'example', 'implementation', 'addProtectedC',
                     'addPublicC', 'addIncludesC', 'cName', 'cplusplusName',
                     'addConstructor', 'cppText'])


def Descriptions(path):
    """every string literal of a definition file, with the keyword it is passed to and the line it
    starts on; the keyword is None for a positional argument or a plain assignment"""
    tree = ast.parse(io.open(path, encoding='utf-8').read())
    keywordOf = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and isinstance(node.value, ast.Constant):
            keywordOf[id(node.value)] = node.arg
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            yield (keywordOf.get(id(node), None), node.value, node.lineno)


def CheckHeadings(paths):
    """the level a heading is written at, and the spelling of a recurring title"""
    spellings = dict((title.casefold(), title) for title in RECURRING_HEADINGS)
    findings = []
    for path in paths:
        for (keyword, text, lineno) in Descriptions(path):
            if keyword in CODE_KEYWORDS:
                continue                     #Python, not prose: a '#' there is a comment
            wanted = HEADING_LEVEL.get(keyword, DEFAULT_HEADING_LEVEL)
            inCode = False
            for (offset, line) in enumerate(text.split('\n')):
                stripped = line.strip()
                listing = re.search(r'\\(begin|end)\{lstlisting\}', stripped)
                if stripped.startswith('```') or listing is not None:
                    inCode = not inCode if listing is None else listing.group(1) == 'begin'
                    continue
                match = None if inCode else re.match(r'^\s*(#+) (.*)$', line)
                if match is None or line.lstrip().startswith('#' * 7):
                    continue
                (level, title) = (len(match.group(1)), match.group(2).strip())
                where = (path, lineno + offset)
                if level != wanted:
                    findings.append(where + ('a heading of ' + str(keyword) + ' is written with '
                                             + str(wanted) + ' "#", not ' + str(level)
                                             + ': ' + title,))
                known = spellings.get(title.casefold())
                if known is not None and known != title:
                    findings.append(where + ('the heading "' + title + '" is the recurring "'
                                             + known + '" spelled differently',))
    return findings


#a citation is [CITE:Key], and the converter turns it into the [Key] that conf.py resolves - it
#appends a Markdown link definition for every key of the bibliography to every document Sphinx
#reads. The marker is what makes a citation checkable: without it, no pattern could tell a key from
#the other square brackets a description is full of ([SI:kg], [0,0,0], [localIndex]) - 1228 of them
#in definitions/ - and twelve of the bibliography's own keys are not shaped like a key at all.
#The third check is for a writer who forgets the marker entirely; 0.85 is chosen from the
#measurement that the closest a non-citation comes to a key is 0.59.
CITATION_SIMILARITY = 0.85
citationMarked = re.compile(r'\[CITE:([^\]]*)\]')
citationToken = re.compile(r'(?<!\])\[([A-Za-z][A-Za-z0-9_.\-]{3,60})\](?!\()')


def BibliographyKeys(path):
    return set(re.findall(r'@\w+\{([^,]+),', io.open(path, encoding='utf-8',
                                                     errors='replace').read()))


def CheckCitations(paths, known):
    """a [CITE:Key] whose key is not in the bibliography, a citation written without the marker,
    and a bracketed token that is nearly a key and therefore probably meant to be one"""
    findings = []
    for path in paths:
        for (keyword, text, lineno) in Descriptions(path):
            if keyword in CODE_KEYWORDS:
                continue

            def Where(match, text=text, lineno=lineno):
                return (path, lineno + text.count('\n', 0, match.start()))

            for match in citationMarked.finditer(text):
                key = match.group(1).strip()
                if key in known:
                    continue
                close = difflib.get_close_matches(key, known, n=1, cutoff=0.6)
                findings.append(Where(match) + ('[CITE:' + key + '] is in no bibliography entry'
                                                + ('; did you mean [CITE:' + close[0] + ']?'
                                                   if close else ''),))
            for match in citationToken.finditer(text):
                token = match.group(1)
                if token in known:
                    findings.append(Where(match) + ('[' + token + '] is a bibliography key and'
                                                    ' has to be written [CITE:' + token + ']',))
                    continue
                close = difflib.get_close_matches(token, known, n=1, cutoff=CITATION_SIMILARITY)
                if close:
                    findings.append(Where(match) + ('[' + token + '] is in no bibliography entry;'
                                                    ' did you mean [CITE:' + close[0] + ']?',))
    return findings


equationLabel = re.compile(r'(?m)^\s*\$\$\s*\(([^)]+)\)\s*$')
markdownLink = re.compile(r'\[[^\]]*\]\(#([^)]+)\)')


def CheckEquationReferences(paths):
    """a reference to an equation is the {eq} role, not a Markdown link

    Both render the same number in the HTML. The LaTeX writer, however, gives a link to an equation
    the anchor "<document>:equation-<label>" while it labels the equation itself
    "equation:<document>:<label>" - so every such link left the PDF as an undefined reference, 42 of
    them, and only the PDF said so (#2655, RG3.14.3)."""
    labels = set()
    for path in paths:
        labels |= set(equationLabel.findall(io.open(path, encoding='utf-8').read()))

    findings = []
    for path in paths:
        for (keyword, text, lineno) in Descriptions(path):
            if keyword in CODE_KEYWORDS:
                continue
            for match in markdownLink.finditer(text):
                if match.group(1) not in labels:
                    continue
                findings.append((path, lineno + text.count('\n', 0, match.start()),
                                 match.group(0) + ' points at an equation; write it as the role, '
                                 '{eq}`' + match.group(1) + '`'))
    return findings


mathSpan = [r'(?<!\\)\$\$.*?\$\$', r'(?<!\\)\$(?:\\.|[^$\\])*\$']


def CheckPercentComments(paths):
    """a comment is <!-- ... --> ; a '%' is a comment only INSIDE mathematics (#2663, RG3.17)

    The reason is StripComments: it runs before the mathematics is protected, so a '%' it took for a
    comment truncated the rest of its line whatever that line was - which is how the marker velocity
    row of MarkerSuperElementRigid lost the tail of its formula. Two spellings stay: the
    %%RSTCOMPATIBLE marker that itemDocsEmitter splits the text on, and an escaped percent sign."""
    findings = []
    for path in paths:
        for (keyword, text, lineno) in Descriptions(path):
            if keyword in CODE_KEYWORDS:
                continue
            #a % inside an HTML comment is already commented out
            spans = [(m.start(), m.end())
                     for m in re.finditer(r'<!--.*?-->', text, flags=re.S)]
            for pattern in mathSpan:
                spans += [(m.start(), m.end()) for m in re.finditer(pattern, text, flags=re.S)]
            for match in re.finditer('%', text):
                index = match.start()
                if index > 0 and text[index - 1] == chr(92):
                    continue                     #an escaped percent sign
                if any(start <= index < end for (start, end) in spans):
                    continue                     #mathematics: the engine reads it
                start = text.rfind(chr(10), 0, index) + 1
                end = text.find(chr(10), index)
                line = text[start:end if end != -1 else len(text)]
                if 'RSTCOMPATIBLE' in line:
                    continue
                findings.append((path, lineno + text.count(chr(10), 0, index),
                                 "a '%' outside mathematics: a comment is <!-- ... -->, and a "
                                 'percent sign is written with a backslash before it'))
    return findings


structuralMacro = re.compile(r'(?<!\\)\\([A-Za-z]+)')


def CheckNoLatex(paths):
    """a backslash command in a description, outside mathematics, is an error (#2655, RG3.14.7.6)

    This is the rule the whole of RG3.14 was for: the description of an item, a structure or a pybind
    call is Markdown, the mathematics inside it is LaTeX, and nothing else is. Until now a macro the
    converter did not know was carried through to the page as itself; a ReportUnknown that
    would have found them sat in latexToMarkdown and was called by nothing, and is deleted.

    What is not a description and is skipped: a value passed to one of CODE_KEYWORDS, which is C++ or
    Python (the \\n of a lambda in cName is a C++ string escape); an HTML comment, which never reaches
    a page; a fenced code block; and the mathematics itself."""
    findings = []
    for path in paths:
        for (keyword, text, lineno) in Descriptions(path):
            if keyword in CODE_KEYWORDS:
                continue
            stripped = re.sub(r'<!--.*?-->', ' ', text, flags=re.S)
            stripped = re.sub(r'(?m)^[ ]*```.*?^[ ]*```', ' ', stripped, flags=re.S)
            for pattern in mathSpan:
                stripped = re.sub(pattern, ' ', stripped, flags=re.S)
            for match in structuralMacro.finditer(stripped):
                findings.append((path, lineno + stripped.count('\n', 0, match.start()),
                                 'a LaTeX command outside mathematics: ' + chr(92)
                                 + match.group(1)))
    return findings


def CheckRawStrings(paths):
    """a literal whose value carries a backslash or mathematics is written r'...'

    Python reads '\\theta' as a tab followed by "heta", and nothing says so: the page shows the tab
    and the formula is gone. The writers of definitions/ paid for this by hand - 158 literals held
    a DOUBLED backslash before RG3.14.9 - which works and is unreadable. The rule has no exception,
    so the check is a rule about the source text, not about the value: the literal's own spelling."""
    findings = []
    for path in paths:
        source = io.open(path, encoding='utf-8').read()
        lines = source.split('\n')
        for node in ast.walk(ast.parse(source)):
            if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
                continue
            if '\\' not in node.value and '$' not in node.value:
                continue
            prefix = lines[node.lineno - 1][max(0, node.col_offset):node.col_offset + 2]
            if prefix[:1].lower() == 'r':
                continue
            findings.append((path, node.lineno,
                             "carries a backslash or mathematics and is not an r'...' literal"))
    return findings


def CheckAbbreviations(paths, declared):
    """ABRV:KEY with a key the list does not have"""
    findings = []
    for path in paths:
        for (_, text, lineno) in Descriptions(path):
            for match in re.finditer(r'\bABRV:([A-Za-z0-9]+)', text):
                if match.group(1) in declared:
                    continue
                line = lineno + text.count('\n', 0, match.start())
                findings.append((path, line, 'ABRV:' + match.group(1)
                                 + ' - no such abbreviation'))
    return findings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    parser.add_argument('--quiet', action='store_true', help='print only findings')
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths = sorted(glob.glob(os.path.join(root, 'definitions', '*.py')))
    declared = DeclaredAbbreviations(os.path.join(root, 'tools', 'generators',
                                                  'examplesDocsEmitter.py'))
    known = BibliographyKeys(os.path.join(root, 'docs', 'bibliographyDoc.bib'))
    findings = (CheckAbbreviations(paths, declared) + CheckHeadings(paths)
                + CheckCitations(paths, known) + CheckRawStrings(paths)
                + CheckPercentComments(paths)
                + CheckEquationReferences(paths) + CheckNoLatex(paths))

    if len(findings) == 0:
        if not args.quiet:
            print('OK: the descriptions of ' + str(len(paths)) + ' definition files use only the '
                  + str(len(declared)) + ' abbreviations that the list has,\n'
                  '    write every heading at the level of the page it is placed in,\n'
                  '    cite only keys of the ' + str(len(known)) + '-entry bibliography,\n'
                  "    and write every literal that carries a backslash as r'...'.")
        return 0

    print('FINDINGS in definitions/ - see definitions/README.md, "Writing a description":')
    for (path, line, what) in findings:
        print('   ' + os.path.relpath(path, root).replace(os.sep, '/') + ':' + str(line)
              + '  ' + what)
    return 1 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
