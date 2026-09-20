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
import os
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
    def Escaped(i):
        """a brace is escaped only after an ODD number of backslashes: \\{ is escaped, \\\\{ is a
        LaTeX line break followed by an ordinary brace"""
        backslashes = 0
        while i - 1 - backslashes >= 0 and text[i - 1 - backslashes] == '\\':
            backslashes += 1
        return backslashes % 2 == 1

    depth = 0
    for i in range(start, len(text)):
        if text[i] == '{' and not Escaped(i):
            depth += 1
        elif text[i] == '}' and not Escaped(i):
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
        if result == '' and line.strip() != '':
            continue            #a line that was nothing but a comment; a BLANK line is kept,
                                #because in Markdown it separates paragraphs
        outLines += [result.rstrip()]
    return '\n'.join(outLines)


def ResolveRSTSwitches(text):
    """\\onlyRST{..} keeps its content, \\ignoreRST{..} drops it - the LaTeX-only branch dies with
    the PDF (info document D8)"""
    text = ReplaceCommand(text, 'onlyRST', 1, lambda a: a)
    text = ReplaceCommand(text, 'ignoreRST', 1, lambda a: '')
    return text


def DropLatexFigures(text):
    """A \\begin{figure} .. \\end{figure} environment is the LaTeX half of a figure whose other
    half is the RST/Markdown one in the \\onlyRST branch - in introduction.tex the tikz pictures
    are written bare rather than inside \\ignoreRST. They are dropped, and each dropped caption is
    PRINTED, because a figure that vanishes without a word is exactly the failure this tool must
    not have (revision2026 step R7.1.5). The tikz sources themselves go with step R7.1.9, which
    replaces them by mermaid."""
    def Drop(match):
        caption = re.search(r'\\caption\{(.{0,60})', match.group(0), flags=re.S)
        print('   dropped LaTeX figure: ' +
              (caption.group(1).replace(chr(10), ' ') if caption is not None else '(no caption)'))
        return ''

    return re.sub(r'\\begin\{figure\}(?:\[[^\]]*\])?.*?\\end\{figure\}', Drop, text, flags=re.S)


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
    """\\bi .. \\item .. \\ei -> a Markdown list, \\ben/\\een -> a numbered one. The INNERMOST list is
    converted first, so that a list inside an \\item becomes an indented sub-list instead of
    swallowing its parent's \\ei (which is what a non-greedy outer match does)."""
    def Convert(match):
        bullet = '-' if match.group(1) == 'i' else '1.'
        items = [item for item in re.split(r'\\item(?![A-Za-z])', match.group(2))
                 if item.strip() != '']
        lines = []
        for (number, item) in enumerate(items):
            item = re.sub(r'^\s*\[[^\]]*\]', '', item)   #\item[] and \item[label]
            mark = bullet if bullet == '-' else str(number + 1) + '.'
            own = []                        #the item's own text, as one line
            block = []                      #sub-lists and code fences, kept as their own lines
            inFence = False
            for line in item.split('\n'):
                if line.strip().startswith('```'):
                    #a code fence inside an item: indented by 2, so that it stays part of the item
                    inFence = not inFence
                    block += ['  ' + line.strip()]
                elif inFence:
                    block += ['  ' + line]      #code, verbatim
                elif line.lstrip()[:2] in ['- ', '1.'] or (len(block) != 0 and line.startswith('  ')):
                    block += ['  ' + line.strip() if not line.startswith('  ') else '  ' + line]
                elif line.strip() != '':
                    own += [line.strip()]
            lines += [mark + ' ' + ' '.join(own)]
            if len(block) != 0:
                lines += [''] + block + ['']
        return '\n' + '\n'.join(lines) + '\n'

    #(?!\\b[in]\b) makes the match stop at the first inner list, i.e. picks the innermost one
    pattern = re.compile(r'\\b([in])\b((?:(?!\\b[in]\b).)*?)\\e[in]\b', re.S)
    while True:
        (text, count) = pattern.subn(Convert, text)
        if count == 0:
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
        #every \rowTable... variant: Three, Four, Five, ... - the cell count is read from the
        #braces that follow, so the name does not matter
        for rowMatch in re.finditer(r'\\rowTable[A-Za-z]*\s*', body):
            index = rowMatch.end()
            cells = []
            while index < len(body) and body[index] == '{':
                (cell, index) = ReadArgument(body, index)
                cells += [re.sub(r'\s*\n\s*', ' ', cell).strip()]
            rows += [cells]
        if len(rows) == 0:
            return ''
        consumed = re.sub(r'\\rowTable[A-Za-z]*\s*(?:\{(?:[^{}]|\{[^{}]*\})*\})*', '', body)
        leftOver = re.findall(r'\\[A-Za-z]+', re.sub(r'(?<!\\)\$(?:\\.|[^$\\])*\$', '', consumed))
        if len(leftOver) != 0:  #a macro inside a table that is not a row: say so, never drop it
            print('WARNING: unhandled inside a table: ' + ' '.join(sorted(set(leftOver))))
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


def ConvertListings(text):
    """\\pythonstyle\\begin{lstlisting} .. \\end{lstlisting} -> a fenced Python block; the
    content is code and must not be touched by any other pass, so this runs before them"""
    def Block(match):
        style = match.group(1) or ''
        options = match.group(2) or ''
        #\plainlststyle is used for CONSOLE OUTPUT, which is not Python and must not be
        #highlighted as if it were
        language = '' if 'plain' in style else 'python'
        if 'language=' in options and 'Python' not in options:
            language = ''                 #a listing that says it is something else
        body = match.group(3).strip(chr(10))
        return eol_ + '```' + language + chr(10) + body + chr(10) + '```' + eol_

    eol_ = chr(10)
    pattern = (r'(?:\\([a-zA-Z]*lststyle|pythonstyle)\s*)?\\begin\{lstlisting\}(\[[^\]]*\])?(.*?)\\end\{lstlisting\}')
    return re.sub(pattern, Block, text, flags=re.S)


def ImagePath(image):
    """the image as the .tex writes it, from the repository root and often without an
    extension (LaTeX picks one); the document needs a source-relative path WITH the extension
    that is actually on disk"""
    image = image.strip().lstrip('/')
    #some .tex files write the path relative to docs/theDoc/ instead of from the root
    if not os.path.exists(image) and os.path.exists('docs/theDoc/' + image):
        image = 'docs/theDoc/' + image
    if not os.path.exists(image) and os.path.exists('docs/theDoc/' + image + '.png'):
        image = 'docs/theDoc/' + image
    if os.path.splitext(image)[1] == '':
        for extension in ['.png', '.jpg', '.jpeg', '.svg', '.pdf']:
            if os.path.exists(image + extension):
                image = image + extension
                break
        else:
            print('   WARNING: no image file found for ' + image)
            image = image + '.png'
    return '/' + image


def LatexRSTFigure(image, label, texWidth, pixels, caption):
    """\\LatexRSTfigure{image}{label}{LaTeX width}{pixel width}{caption}: of the two widths only
    the pixel one survives, the LaTeX one dies with the PDF (info document D8)"""
    return (chr(10) + '(' + RefLabel(label) + ')=' + chr(10) +
            '```{figure} ' + ImagePath(image) + chr(10) +
            ':width: ' + pixels.strip() + chr(10) + chr(10) +
            ' '.join(caption.split()) + chr(10) + '```' + chr(10))


def ConvertRSTFigures(text):
    """the \\onlyRST branches contain RST directives written by hand; a figure becomes the MyST
    figure directive, with the label above it as a MyST target"""
    def Block(match):
        (label, image, options, caption) = match.groups()
        lines = ['']
        if label is not None:
            lines += ['(' + RefLabel(label) + ')=']
        lines += ['```{figure} ' + ImagePath(image)]
        for option in re.findall(r':([a-z]+):\s*(\S+)', options or ''):
            lines += [':' + option[0] + ': ' + option[1]]
        lines += ['', caption.strip(), '```', '']
        return chr(10).join(lines)

    pattern = (r'(?:\.\.\s+_([^:\n]+):\s*\n)?\.\.\s+figure::\s*(\S+)\s*\n'
               r'((?:\s+:[a-z]+:[^\n]*\n)*)\s*\n(\s+[^\n]+)\n')
    return re.sub(pattern, Block, text)


def ConvertInline(text):
    """the one-argument text macros, and the ones that take none"""
    text = ReplaceCommand(text, 'texttt', 1, lambda a: '`' + a.replace('\\_', '_').replace('\\&', '&') + '`')
    text = ReplaceCommand(text, 'mybold', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'textbf', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'myitalics', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'textit', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'noindent', 0, lambda: '')
    #abbreviations: the target lives in the generated abbreviation page
    for name in ['hac', 'hacs', 'acf', 'acl', 'acs', 'ac']:
        #an abbreviation target is a bare label, not a section: {ref} without explicit text
        #cannot find a title for it, and the strict build calls that an error
        text = ReplaceCommand(text, name, 1,
                              lambda a: '{ref}`' + a.strip() + ' <' + a.strip() + '>`')
    text = ReplaceCommand(text, 'refSectionA', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'refSection', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = re.sub(r'\\lbrack(?![A-Za-z])', '[', text)
    text = re.sub(r'\\rbrack(?![A-Za-z])', ']', text)
    #\LatexRSTfigure{image}{label}{LaTeX width}{pixel width}{caption}: the two widths are for
    #the two outputs, and only the pixel one survives
    text = ReplaceCommand(text, 'LatexRSTfigure', 5, LatexRSTFigure)
    for name in ['eqq', 'eqref']:
        text = ReplaceCommand(text, name, 1, lambda a: '{eq}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'ref', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'fig', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'exuUrl', 2, lambda url, name: '[' + name.strip() + '](' + url.strip() + ')')
    text = ReplaceCommand(text, 'newpage', 0, lambda: '')
    text = ReplaceCommand(text, 'clearpage', 0, lambda: '')
    text = ReplaceCommand(text, 'rstStartNewLine', 0, lambda: '')
    #a rule between the parts of a tutorial: a thematic break says the same in Markdown
    text = ReplaceCommand(text, 'horizontalRuler', 0, lambda: '\n---\n')
    text = ReplaceCommand(text, 'eq', 1, lambda a: '{eq}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'refChapter', 1, lambda a: '{ref}`' + RefLabel(a.strip()) + '`')
    text = ReplaceCommand(text, 'footnote', 1, lambda a: ' (' + a.strip() + ')')
    #the colour macros of the contact chapter are MathJax macros (conf.py); the few occurrences
    #OUTSIDE math are prose that names the colour itself
    for name in ['termA', 'termC']:
        text = ReplaceCommand(text, name, 1, lambda a: a.strip())
    #citations are DROPPED by the current LaTeX->RST path, which is why the HTML has sentences
    #ending in "by the main developer ." - keep them readable until #2550 gives them a page
    text = ReplaceCommand(text, 'cite', 1,
                          lambda a: '[' + '; '.join(key.strip() for key in a.split(',')) + ']')
    text = ReplaceCommand(text, 'label', 1, lambda a: '\n(' + RefLabel(a.strip()) + ')=\n')
    text = ReplaceCommand(text, 'vspace', 1, lambda a: '')
    text = ReplaceCommand(text, 'hspace', 1, lambda a: ' ')
    text = re.sub(r'\\codeName\b', 'Exudyn', text)
    #macros that expand to WORDS, not to symbols; inside math MathJax knows them (conf.py),
    #outside it they have to be written out
    words = {'SON': '$2^\\mathrm{nd}$ order differential equations',
             'FON': '$1^\\mathrm{st}$ order differential equations',
             'AEN': 'algebraic equations',
             'SYSN': 'number of coordinates of the system equations',
             'textdegree':-1}
    for (name, replacement) in words.items():
        if replacement == -1:
            replacement = chr(176)
        #a function as the replacement: the text has backslashes of its own
        text = re.sub(r'\\' + name + r'(?![A-Za-z])', lambda m, r=replacement: r, text)
    text = re.sub(r'\\ ', ' ', text)
    text = re.sub(r'\\\\?(?=\\s*$)', '', text, flags=re.M)   #a trailing \\ or \\ at a line end
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
    text = re.sub(r'^[ ]*```.*?^[ ]*```', '', text, flags=re.S | re.M)  #code is not LaTeX
    (stripped, _) = ProtectMath(text)
    found = {}
    for match in re.finditer(r'\\([A-Za-z]+)', stripped):
        found[match.group(1)] = found.get(match.group(1), 0) + 1
    return found


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ConvertText(text):
    """One piece of running text, not a file: the description of a parameter, a class or a
    function as `definitions/` and the docstrings write it. Used by the documentation emitters
    of revision2026 step R7.1.6, which have the same LaTeX to convert as the chapters did - so
    they call this rather than growing a second converter.

    Math is left alone (conf.py declares the macros to MathJax); everything else that the
    project writes in prose is turned into Markdown."""
    text = StripComments(text)
    text = ResolveRSTSwitches(text)
    text = ConvertDisplayMath(text)
    (text, pieces) = ProtectMath(text)
    text = ConvertSections(text)
    text = ConvertTables(text)
    text = ConvertLists(text)
    text = ConvertInline(text)
    #helpers that exist only to lay out a LaTeX table
    text = re.sub(r'\\tabnewline\s*', '', text)
    text = RestoreMath(text, pieces)
    text = re.sub(r'(?m)^[ \t]+', '', text)          #no stray indentation from the .tex source
    text = re.sub(r'\n{3,}', '\n\n', text)
    return text.strip()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Convert(text):
    text = StripComments(text)
    text = ConvertListings(text)        #code first: nothing else may touch its content
    text = ResolveRSTSwitches(text)
    text = DropLatexFigures(text)       #after the switches: what is left is LaTeX-only
    text = ConvertRSTFigures(text)
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
