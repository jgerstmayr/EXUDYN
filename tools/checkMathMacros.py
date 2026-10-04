#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkMathMacros - every macro used in the documentation's math is declared to MathJax
#
# Why this check exists (#2549): the math of the manual is written with
# the project's own macros - $\qv\cConfig$, $\LU{0b}{\Rot}$, $\diffmOI{f_c}$ - and MathJax only
# resolves what conf.py declares in mathjax3_config. A macro that is NOT declared renders as raw
# LaTeX source in the browser, and NOTHING reports it: Sphinx does not read math, MathJax runs in
# the reader's browser, and the page builds without a warning. That is exactly what happened when
# the chapters moved from .tex to .md - the old .tex -> .rst converter used to EXPAND the macros,
# so conf.py had only 118 of the document's 204 and no one could notice.
#
# Usage:
#   python tools/checkMathMacros.py            #report
#   python tools/checkMathMacros.py --check    #exit 1 if a macro is missing (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import glob
import io
import re
import sys

#LaTeX and AMS commands MathJax knows by itself; a macro is only "missing" if it is none of these
BUILT_IN = set("""
    alpha beta gamma delta epsilon varepsilon zeta eta theta vartheta iota kappa lambda mu nu xi pi
    varpi rho varrho sigma varsigma tau upsilon phi varphi chi psi omega Gamma Delta Theta Lambda Xi
    Pi Sigma Upsilon Phi Psi Omega
    frac sqrt sum prod int oint lim limits sup inf max min det dim ker deg gcd hom arg exp ln log
    sin cos tan cot sec csc arcsin arccos arctan sinh cosh tanh coth
    left right big Big bigg Bigg langle rangle lbrack rbrack lbrace rbrace vert Vert lvert rvert
    lfloor rfloor lceil rceil
    mathbf mathrm mathit mathcal mathbb mathsf mathtt mathfrak boldsymbol bm text textrm textbf
    textit texttt operatorname displaystyle scriptstyle
    dot ddot hat bar tilde vec widehat widetilde overline underline overbrace underbrace check acute
    grave breve
    cdot cdots ldots vdots ddots times div pm mp ast star circ bullet oplus ominus otimes odot
    approx equiv sim simeq cong neq ne leq le geq ge ll gg propto subset supset subseteq supseteq in
    notin ni forall exists nabla partial infty emptyset angle perp parallel
    rightarrow leftarrow Rightarrow Leftarrow leftrightarrow Leftrightarrow implies impliedby iff to
    mapsto longrightarrow
    quad qquad hspace vspace space phantom hphantom vphantom
    begin end array matrix pmatrix bmatrix vmatrix cases aligned align split nonumber label tag
    color textcolor colorbox
    substack choose binom over atop stackrel overset underset
    mbox prime rm bf it sf tt backslash not
    lVert rVert mathrel
    """.split())


def DeclaredMacros(confPath):
    text = io.open(confPath, encoding='utf-8').read()
    block = text[text.index("'macros': {"):]
    return set(re.findall(r"^\s*'([A-Za-z]+)':", block, flags=re.M))


def UsedMacros(paths):
    """every macro inside $...$ or $$...$$, outside fenced code"""
    used = {}
    for path in paths:
        text = io.open(path, encoding='utf-8').read()
        text = re.sub(r'(?m)^[ ]*```.*?^[ ]*```', '', text, flags=re.S)
        for piece in re.findall(r'(?<!\\)\$\$.*?\$\$|(?<!\\)\$(?:\\.|[^$\\])*\$', text, flags=re.S):
            for name in re.findall(r'\\([A-Za-z]+)', piece):
                used.setdefault(name, []).append(path)
    return used


def BlankLinesInDisplayMath(paths):
    """the display formulas ($$...$$) that contain a blank line: Sphinx splits a formula at a blank line
    into separate equations, which tears an aligned environment apart - the page then shows
    '\\begin{aligned} ended with \\end{split}' instead of the formula (#2834)"""
    found = []
    for path in paths:
        text = io.open(path, encoding='utf-8').read()
        text = re.sub(r'(?m)^[ ]*```.*?^[ ]*```', '', text, flags=re.S)
        for piece in re.findall(r'(?<!\\)\$\$.*?\$\$', text, flags=re.S):
            if re.search(r'\n[ \t]*\n', piece.strip('$').strip('\n')):
                found.append((path, piece.strip('$').strip()[:60]))
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 when a macro is missing')
    args = parser.parse_args()

    declared = DeclaredMacros('conf.py')
    #the hand-written chapters and,, the emitter output: the item
    #reference manual carries most of the document's math
    used = UsedMacros(sorted(glob.glob('docs/manual/*.md')
                             + glob.glob('docs/generated/**/*.md', recursive=True)))
    missing = {name: files for (name, files) in used.items()
               if name not in declared and name not in BUILT_IN}

    blank = BlankLinesInDisplayMath(sorted(glob.glob('docs/manual/*.md') + glob.glob('docs/dev/*.md')
                                           + glob.glob('docs/howTo/*.md')
                                           + glob.glob('docs/generated/**/*.md', recursive=True)))
    if blank:
        print('BLANK LINES inside display math - Sphinx splits the formula there; remove them:')
        for (path, start) in blank:
            print('   ' + path + ': ' + start)
        if args.check:
            return 1

    if len(missing) == 0:
        print('OK: all ' + str(len(used)) + ' macros used in the documentation math are known to '
              'MathJax\n    (' + str(len(declared)) + ' declared in conf.py).')
        return 0

    print('MISSING from mathjax3_config in conf.py - these render as raw LaTeX in the browser:')
    for name in sorted(missing, key=lambda n: -len(missing[n])):
        files = sorted(set(missing[name]))
        print('   \\' + name + '  x' + str(len(missing[name])) + '  ' +
              ', '.join(file.split('/')[-1] for file in files[:3]))
    print('\nAdd them to the macros dict in conf.py.')
    return 1 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
