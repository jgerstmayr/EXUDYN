#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# tex2md - convert one hand-written documentation chapter from LaTeX to MyST Markdown
#
# ONE-SHOT TOOL, revision2026 step R7.1.5 (#2549). It exists to convert the seven chapters of
# docs/theDoc/ once, and it is deleted together with latexConverter.py and doc2rst.py in step
# R7.1.7. It is therefore deliberately not general: it knows the macros THIS documentation uses,
# and it reports every macro it does not know instead of guessing.
#
# The output is a STARTING POINT that is then hand-finished and committed as the new source; the
# converter is not rerun over a finished chapter. The check of a conversion is the generated
# docs/RST/<Chapter>.rst, which is what Sphinx renders today.
#
# Math is NOT converted: conf.py already declares the document's macros to MathJax
# (mathjax3_config), so $...$ carries over verbatim - which is why this file is 300 lines and not
# 3000.
#
# Usage:  python tools/tex2md.py docs/theDoc/notation.tex docs/manual/notation.md
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import re
import sys

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helpers
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def RefLabel(label):
    """a LaTeX label to the target name doc2rst.py generates: lower case, ':' becomes '-'"""
    return label.replace(':', '-').replace('_', '-').lower()


def ReadArgument(text, start):
    """read one {...} argument that starts at text[start]=='{'; returns (content, indexAfter)"""
    assert text[start] == '{'
    depth = 0
    for i in range(start, len(text)):
        if text[i] == '{' and (i == 0 or text[i - 1] != '\\'):
            depth += 1
        elif text[i] == '}' and text[i - 1] != '\\':
            depth -= 1
            if depth == 0:
                return (text[start + 1:i], i + 1)
    raise ValueError('unbalanced { at ' + str(start) + ': ' + text[start:start + 60])


def ReplaceCommand(text, command, nArgs, build):
    """replace every \\command{..}..{..} by build(args); innermost-last, so nesting works"""
    pattern = re.compile(r'\\' + command + r'(?![A-Za-z])')
    while True:
        match = pattern.search(text)
        if match is None:
            return text
        index = match.end()
        args = []
        for _ in range(nArgs):
            while index < len(text) and text[index] in ' \t\n':
                index += 1
            if index >= len(text) or text[index] != '{':
                raise ValueError('\\' + command + ' without argument: ' + text[match.start():match.start() + 60])
            (argument, index) = ReadArgument(text, index)
            args += [argument]
        text = text[:match.start()] + build(*args) + text[index:]


def ProtectMath(text):
    """math is carried over verbatim, so it must not be touched by the text replacements;
    returns (textWithPlaceholders, pieces)"""
    pieces = []

    def Store(match):
        pieces.append(match.group(0))
        return '\x00MATH' + str(len(pieces) - 1) + '\x00'

    #display math first (written by ConvertDisplayMath), then inline $..$
    text = re.sub(r'(?<!\\)\$\$.*?\$\$', Store, text, flags=re.S)
    text = re.sub(r'(?<!\\)\$(?:\\.|[^$\\])*\$', Store, text, flags=re.S)
    return (text, pieces)


def RestoreMath(text, pieces):
    for (i, piece) in enumerate(pieces):
        text = text.replace('\x00MATH' + str(i) + '\x00', piece)
    return text


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the passes
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def StripComments(text):
    """remove LaTeX comments, keeping \\% ; a comment-only line disappears entirely"""
    outLines = []
    for line in text.split('\n'):
        result = ''
        i = 0
        while i < len(line):
            if line[i] == '%' and (i == 0 or line[i - 1] != '\\'):
                break
            result += line[i]
            i += 1
        if i == 0 and result == '':
            continue            #a line that was nothing but a comment
        outLines += [result.rstrip()]
    return '\n'.join(outLines)


def ResolveRSTSwitches(text):
    """\\onlyRST{..} keeps its content, \\ignoreRST{..} drops it - the LaTeX-only branch dies with
    the PDF (info document D8)"""
    text = ReplaceCommand(text, 'onlyRST', 1, lambda a: a)
    text = ReplaceCommand(text, 'ignoreRST', 1, lambda a: '')
    return text


def ConvertSections(text):
    """\\mysectionlabel{T}{l} -> a MyST target plus a heading; the depth follows the macro name"""
    levels = [('mysubsubsubsection', 5), ('mysubsubsection', 4), ('mysubsection', 3),
              ('mysection', 2), ('chapter', 1)]
    for (name, depth) in levels:
        text = ReplaceCommand(text, name + 'label', 2,
                              lambda title, label, d=depth:
                              '\n(' + RefLabel(label) + ')=\n' + '#' * d + ' ' + title.strip() + '\n')
        text = ReplaceCommand(text, name, 1,
                              lambda title, d=depth: '\n' + '#' * d + ' ' + title.strip() + '\n')
    return text


def ConvertLists(text):
    """\\bi .. \\item .. \\ei -> a Markdown list; \\ben/\\een -> a numbered one"""
    def Convert(match, bullet):
        body = match.group(1)
        items = [item.strip() for item in re.split(r'\\item(?![A-Za-z])', body) if item.strip() != '']
        lines = []
        for (i, item) in enumerate(items):
            mark = bullet if bullet != '1.' else str(i + 1) + '.'
            item = re.sub(r'\s*\n\s*', ' ', item).strip()
            lines += [mark + ' ' + item]
        return '\n' + '\n'.join(lines) + '\n'

    text = re.sub(r'\\bi\b(.*?)\\ei\b', lambda m: Convert(m, '-'), text, flags=re.S)
    text = re.sub(r'\\ben\b(.*?)\\een\b', lambda m: Convert(m, '1.'), text, flags=re.S)
    return text


def ConvertDisplayMath(text):
    """\\be .. \\ee -> $$ .. $$ , \\bea .. \\eea -> $$ \\begin{aligned} .. \\end{aligned} $$ ;
    a \\label becomes the MyST equation target"""
    def Block(body, aligned):
        label = None
        match = re.search(r'\\label\{([^}]*)\}', body)
        if match is not None:
            label = RefLabel(match.group(1))
            body = body[:match.start()] + body[match.end():]
        body = body.replace('\\eqComma', '\\, ,').replace('\\eqDot', '\\, .')
        body = body.strip('\n')
        if aligned:
            body = body.replace('&=&', '&=').replace('\\\\\n', '\\\\\n')
            body = '\\begin{aligned}\n' + body + '\n\\end{aligned}'
        out = '\n$$\n' + body + '\n$$'
        if label is not None:
            out += ' (' + label + ')'
        return out + '\n'

    text = re.sub(r'\\bea\b(.*?)\\eea\b', lambda m: Block(m.group(1), True), text, flags=re.S)
    text = re.sub(r'\\be\b(.*?)\\ee\b', lambda m: Block(m.group(1), False), text, flags=re.S)
    return text


def ConvertTables(text):
    """\\startTable / \\startGenericTable .. \\rowTable* .. \\finishTable -> a Markdown table;
    the first row is the header"""
    def Convert(match):
        body = match.group(2)
        rows = []
        for rowMatch in re.finditer(r'\\rowTable(Three|)\s*', body):
            index = rowMatch.end()
            cells = []
            while index < len(body) and body[index] == '{':
                (cell, index) = ReadArgument(body, index)
                cells += [re.sub(r'\s*\n\s*', ' ', cell).strip()]
            rows += [cells]
        if len(rows) == 0:
            return ''
        width = max(len(row) for row in rows)
        rows = [row + [''] * (width - len(row)) for row in rows]
        #the header of a \startTable is in the macro arguments, a \startGenericTable has it as row 0
        header = match.group(1)
        if header is not None and header.strip().startswith('{') and not header.strip().startswith('{|'):
            index = 0
            headerCells = []
            stripped = header.strip()
            while index < len(stripped) and stripped[index] == '{':
                (cell, index) = ReadArgument(stripped, index)
                headerCells += [cell.strip()]
            if len(headerCells) == width:
                rows = [headerCells] + rows
        lines = ['| ' + ' | '.join(cell.replace('|', '\\|') for cell in rows[0]) + ' |',
                 '|' + '---|' * width]
        for row in rows[1:]:
            lines += ['| ' + ' | '.join(cell.replace('|', '\\|') for cell in row) + ' |']
        return '\n' + '\n'.join(lines) + '\n'

    return re.sub(r'\\start(?:Generic)?Table((?:\s*\{(?:[^{}]|\{[^{}]*\})*\})*)(.*?)\\finishTable',
                  Convert, text, flags=re.S)


def ConvertInline(text):
    """the one-argument text macros, and the ones that take none"""
    text = ReplaceCommand(text, 'texttt', 1, lambda a: '`' + a.replace('\\_', '_').replace('\\&', '&') + '`')
    text = ReplaceCommand(text, 'mybold', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'textbf', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'myitalics', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'textit', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'noindent', 0, lambda: '')
    #abbreviations: the target lives in the generated abbreviation page
    for name in ['hac', 'hacs', 'acf', 'acl', 'ac']:
        #an abbreviation target is a bare label, not a section: {ref} without explicit text
        #cannot find a title for it, and the strict build calls that an error
        text = ReplaceCommand(text, name, 1,
                              lambda a: '{ref}`' + a.strip() + ' <' + a.strip() + '>`')
    text = ReplaceCommand(text, 'refSection', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'eq', 1, lambda a: '{eq}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'refChapter', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'footnote', 1, lambda a: ' (' + a.strip() + ')')
    #citations are DROPPED by the current LaTeX->RST path, which is why the HTML has sentences
    #ending in "by the main developer ." - keep them readable until #2550 gives them a page
    text = ReplaceCommand(text, 'cite', 1,
                          lambda a: '[' + '; '.join(key.strip() for key in a.split(',')) + ']')
    text = ReplaceCommand(text, 'label', 1, lambda a: '\n(' + RefLabel(a.strip()) + ')=\n')
    text = ReplaceCommand(text, 'vspace', 1, lambda a: '')
    text = ReplaceCommand(text, 'hspace', 1, lambda a: ' ')
    text = re.sub(r'\\codeName\b', 'Exudyn', text)
    text = re.sub(r'\\ ', ' ', text)
    text = re.sub(r'\\%', '%', text)
    text = re.sub(r'\\&', '&', text)
    text = re.sub(r'\\_', '_', text)
    return text


def NormalizeHeadings(text):
    """a chapter starts at '#' and each heading is at most one level below the one it sits under -
    Sphinx rejects a jump from H1 to H3 under -W. The .tex depths are relative to the whole
    document and are not even locally consistent (notation.tex puts a \\mysubsubsection directly
    under a \\mysection), so this is a walk over the nesting and not a global mapping."""
    result = []
    stack = []                              #the source depths currently open
    for line in text.split('\n'):
        match = re.match(r'^(#+) (.*)$', line)
        if match is None:
            result += [line]
            continue
        depth = len(match.group(1))
        while len(stack) != 0 and stack[-1] >= depth:
            stack.pop()
        stack += [depth]
        result += ['#' * len(stack) + ' ' + match.group(2)]
    return '\n'.join(result)


def Tidy(text):
    text = re.sub(r'[ \t]+\n', '\n', text)
    text = re.sub(r'\$\$\n+', '$$\n', text)
    text = re.sub(r'\n{3,}', '\n\n', text)
    return text.strip() + '\n'


def ReportUnknown(text):
    """every backslash command left outside math - the point of the tool is to name them"""
    (stripped, _) = ProtectMath(text)
    found = {}
    for match in re.finditer(r'\\([A-Za-z]+)', stripped):
        found[match.group(1)] = found.get(match.group(1), 0) + 1
    return found


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Convert(text):
    text = StripComments(text)
    text = ResolveRSTSwitches(text)
    text = ConvertDisplayMath(text)
    (text, pieces) = ProtectMath(text)
    text = ConvertSections(text)
    text = ConvertTables(text)
    text = ConvertLists(text)
    text = ConvertInline(text)
    text = RestoreMath(text, pieces)
    text = NormalizeHeadings(text)
    return Tidy(text)


def main():
    parser = argparse.ArgumentParser(description='convert one .tex chapter to MyST Markdown')
    parser.add_argument('source', help='the .tex file')
    parser.add_argument('target', help='the .md file to write')
    args = parser.parse_args()

    text = io.open(args.source, encoding='utf-8', newline='').read()
    markdown = Convert(text)
    io.open(args.target, 'w', encoding='utf-8', newline='\n').write(markdown)

    unknown = ReportUnknown(markdown)
    print('wrote ' + args.target + ' (' + str(len(markdown.split(chr(10)))) + ' lines)')
    if len(unknown) != 0:
        print('macros left for the hand pass:')
        for name in sorted(unknown, key=lambda n: -unknown[n]):
            print('   \\' + name + '  x' + str(unknown[name]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
