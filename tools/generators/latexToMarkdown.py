#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# latexToMarkdown - the LaTeX that definitions/ and the docstrings are written in, as MyST Markdown
#
# Called by the documentation emitters (autoGenerateHelper.LatexText2Markdown) on every
# generation: an item description, a parameter, a function docstring and an equation block are
# written in the project's LaTeX macros, and the documentation is Markdown since revision2026
# step R7.1.6.
#
# It was tools/tex2md.py, the one-shot tool that converted the nine hand-written chapters in step
# R7.1.5, and it kept the property that made it usable there: it knows the macros THIS
# documentation uses and reports the ones it does not know instead of guessing. The file-conversion
# half went with the .tex files in step R7.1.7; ConvertText is what the emitters call.
#
# Math is NOT converted: conf.py declares the document's macros to MathJax (mathjax3_config), so
# $...$ carries over verbatim - which is why this file is 500 lines and not 3000. Every macro used
# in math is checked by tools/checkMathMacros.py.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import io
import os
import re

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helpers
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def RefLabel(label):
    """a LaTeX label as a MyST target name: lower case, ':' and '_' become '-'"""
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

    def Store(match, marker='MATH'):
        pieces.append(match.group(0))
        return '\x00' + marker + str(len(pieces) - 1) + '\x00'

    #display math first (written by ConvertDisplayMath), then inline $..$; the two get DIFFERENT
    #markers because a later pass has to tell them apart - a display block must keep a line of its
    #own, an inline one must not (see ConvertLists, #2593)
    text = re.sub(r'(?<!\\)\$\$.*?\$\$', lambda m: Store(m, 'DISPLAYMATH'), text, flags=re.S)
    text = re.sub(r'(?<!\\)\$(?:\\.|[^$\\])*\$', Store, text, flags=re.S)
    return (text, pieces)


def RestoreMath(text, pieces):
    for (i, piece) in enumerate(pieces):
        text = text.replace('\x00MATH' + str(i) + '\x00', piece)
        #a display block that stands alone on an indented line - inside a list item - keeps that
        #indentation on every one of its lines, or it falls out of the item (#2593)
        for match in re.findall(r'(?m)^([ ]+)\x00DISPLAYMATH' + str(i) + '\x00[ ]*$', text):
            text = text.replace(match + '\x00DISPLAYMATH' + str(i) + '\x00',
                                match + piece.replace('\n', '\n' + match))
        text = text.replace('\x00DISPLAYMATH' + str(i) + '\x00', piece)
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
    not have. The tikz sources themselves go with step R7.1.9, which
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
                elif line.strip().startswith('\x00DISPLAYMATH'):
                    #a formula of its own inside an item: it must NOT be joined into the item's
                    #text line, where the $$ would end up in the middle of a sentence and MyST
                    #would read the sentence as the formula and the formula as prose (#2593)
                    block += ['  ' + line.strip()]
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
        #\nonumber suppressed the number of an eqnarray row. There is no eqnarray any more: the
        #block becomes $$ .. $$ with \begin{aligned}, which numbers nothing, so MathJax ignores it
        #and LaTeX refuses it ("Missing \cr inserted", #2593)
        body = re.sub(r'\\nonumber\s*', '', body)
        #.strip() and not .strip('\n'): the LaTeX body is indented, so stripping newlines alone
        #leaves a line of blanks before the closing $$ - and a blank line ENDS a display math
        #block in MyST, which left the last line of the formula outside it (#2593)
        body = body.strip()
        if aligned:
            body = body.replace('&=&', '&=').replace('\\\\\n', '\\\\\n')
            body = '\\begin{aligned}\n' + body + '\n\\end{aligned}'
        #a BLANK line before the $$, not just a newline: inside a \item the list conversion joins
        #the item's text into one line, and a single newline there is not enough to keep the $$ at
        #the start of a line - which is what turned formulas into prose and prose into formulas
        #in ObjectContactFrictionCircleCable2D and ObjectConnectorCoordinateSpringDamperExt (#2593)
        out = '\n\n$$\n' + body + '\n$$'
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
        #the listings of the item definitions are indented like the Python source they sit in;
        #that common indentation is not the code's own
        indents = [len(line) - len(line.lstrip()) for line in body.split(chr(10))
                   if line.strip() != '']
        if len(indents) != 0 and min(indents) != 0:
            body = chr(10).join(line[min(indents):] for line in body.split(chr(10)))
        return eol_ + '```' + language + chr(10) + body + chr(10) + '```' + eol_

    eol_ = chr(10)
    pattern = (r'(?:\\([a-zA-Z]*lststyle|pythonstyle)\s*)?\\begin\{lstlisting\}(\[[^\]]*\])?(.*?)\\end\{lstlisting\}')
    return re.sub(pattern, Block, text, flags=re.S)


def ImagePath(image):
    """the image as the .tex writes it, from the repository root and often without an
    extension (LaTeX picks one); the document needs a source-relative path WITH the extension
    that is actually on disk"""
    image = image.strip().lstrip('/')
    #the definitions write an image by its bare name; the figures live in docs/figures/ since
    #revision2026 step R7.1.7
    for prefix in ['docs/figures/', 'docs/']:
        if not os.path.exists(image) and (os.path.exists(prefix + image)
                                          or os.path.exists(prefix + image + '.png')):
            image = prefix + image
            break
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


def ConvertRSTImages(text):
    """The caption-less twin of ".. figure::"; one item description uses it of ".. figure::" and one item description uses it
    (ObjectConnectorRollingDiscPenalty). Without this it stayed in the page as RST text, which
    renders as literal ".. image:: docs/figures/..."."""
    def Block(match):
        (image, options) = match.groups()
        lines = ['', '```{figure} ' + ImagePath(image)]
        for option in re.findall(r':([a-z]+):\s*(\S+)', options or ''):
            lines += [':' + option[0] + ': ' + option[1]]
        lines += ['```', '']
        return chr(10).join(lines)

    newline = chr(10)
    pattern = (r'\.\.\s+image::\s*(\S+)\s*' + newline
               + r'((?:\s+:[a-z]+:[^' + newline + r']*' + newline + r')*)')
    return re.sub(pattern, Block, text)


def ConvertInline(text):
    """the one-argument text macros, and the ones that take none"""
    #the brace form of bold and italics, which the item definitions use: {\bf name}
    text = re.sub(r'\{\\bf\s+([^{}]*)\}', lambda m: '**' + m.group(1).strip() + '**', text)
    text = re.sub(r'\{\\it\s+([^{}]*)\}', lambda m: '*' + m.group(1).strip() + '*', text)
    text = ReplaceCommand(text, 'texttt', 1, lambda a: '`' + a.replace('\\_', '_').replace('\\&', '&') + '`')
    text = ReplaceCommand(text, 'mybold', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'textbf', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'myitalics', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'textit', 1, lambda a: '*' + a.strip() + '*')
    text = ReplaceCommand(text, 'noindent', 0, lambda: '')
    #the user function blocks of the item definitions; the old RST left these as raw LaTeX
    text = ReplaceCommand(text, 'userFunctionExample', 1, lambda a: '*Example*:')
    text = ReplaceCommand(text, 'userFunction', 1,
                          lambda a: '**Userfunction**: `' + a.strip() + '`')
    text = ReplaceCommand(text, 'returnValue', 0, lambda: '**return value**')
    text = ReplaceCommand(text, 'paragraph', 1, lambda a: '**' + a.strip() + '**')
    text = ReplaceCommand(text, 'mysmall', 0, lambda: '')
    text = ReplaceCommand(text, 'phantom', 1, lambda a: '')   #LaTeX spacing, nothing in Markdown
    text = ReplaceCommand(text, 'text', 1, lambda a: a.strip())  #\text outside math is prose
    #a LaTeX line break in running prose; inside math it is protected, and a table cell collapses
    #it back to a space
    text = re.sub(r'\\\\[ \t]*', '  \n', text)
    #\addExampleImage{X} shows docs/figures/X.png next to the item description
    text = ReplaceCommand(text, 'addExampleImage', 1,
                          lambda a: '\n\n```{image} ' + ImagePath('docs/figures/'
                                                                 + a.strip() + '.png')
                                  + '\n:width: 400\n```\n')
    #abbreviations: the target lives in the generated abbreviation page
    for name in ['hac', 'hacs', 'acf', 'acl', 'acs', 'acp', 'ac']:
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
    for name in ['eqq', 'eqref', 'eqs']:
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
    inCode = False                          #a '# comment' of a Python example is not a heading
    for line in text.split('\n'):
        if line.lstrip().startswith('```'):
            inCode = not inCode
        match = None if inCode else re.match(r'^(#+) (.*)$', line)
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
def DedentOutsideCode(text):
    """the .tex sources indent their prose, and that indentation means nothing in Markdown - but
    inside a fenced block it is the Python code's own"""
    lines = []
    inCode = False
    for line in text.split(chr(10)):
        if line.lstrip().startswith('```'):
            inCode = not inCode
            lines += [line.lstrip()]
            continue
        lines += [line if inCode else line.lstrip(' ' + chr(9))]
    return chr(10).join(lines)


def ConvertText(text):
    """One piece of running text, not a file: the description of a parameter, a class or a
    function as `definitions/` and the docstrings write it. Used by the documentation emitters
, which have the same LaTeX to convert as the chapters did - so
    they call this rather than growing a second converter.

    Math is left alone (conf.py declares the macros to MathJax); everything else that the
    project writes in prose is turned into Markdown."""
    text = StripComments(text)
    text = ResolveRSTSwitches(text)
    #the item definitions put their figures into the \onlyRST branch, as RST directives
    text = ConvertRSTFigures(text)
    text = ConvertRSTImages(text)
    #a listing becomes a fenced block first: its content is code and no later pass may touch it
    text = ConvertListings(text)
    text = ConvertDisplayMath(text)
    (text, pieces) = ProtectMath(text)
    text = ConvertSections(text)
    text = ConvertTables(text)
    text = ConvertLists(text)
    text = ConvertInline(text)
    #helpers that exist only to lay out a LaTeX table
    text = re.sub(r'\\tabnewline\s*', '', text)
    text = RestoreMath(text, pieces)
    text = DedentOutsideCode(text)                   #no stray indentation from the .tex source
    text = re.sub(r'\n{3,}', '\n\n', text)
    return text.strip()


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
