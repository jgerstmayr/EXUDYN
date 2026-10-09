# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
# authors: original template file used from sphinx; consistently changed by Johannes Gerstmayr
# data: 2022
# description: this file is used for converting Exudyn .rst files into html documentation using sphinx

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create exudynVersionString
exudynVersionString=''
file='tools/generators/exudynVersion.py'
exec(open(file).read(), globals())

# print('version='+exudynVersionString)

release = exudynVersionString

project = 'Exudyn'+release
copyright = '2023' #'2023, Johannes Gerstmayr'
author = 'Johannes Gerstmayr'
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#try to patch pygments Python style
#seems not to work:
# from confHelper import listClassNames, listFunctionNames

#initialize to avoid error markers
listClassNames=[]
listPyClassNames=[]
listItemNames=[]
listFunctionNames=[]
listPyFunctionNames=[]

file='tools/generators/generated/confHelper.py'
exec(open(file).read(), globals())
file='tools/generators/generated/confHelperItems.py'
exec(open(file).read(), globals())
file='tools/generators/generated/confHelperPyUtilities.py'
exec(open(file).read(), globals())
#the citation keys of docs/bibliographyDoc.bib, for AppendCitationDefinitions() at the end
#of this file
citationKeys=[]
file='tools/generators/generated/confHelperCitations.py'
exec(open(file).read(), globals())

import pygments
from pygments.lexers import PythonLexer
from pygments.lexer import Lexer, RegexLexer, include, bygroups, using, \
    default, words, combined, do_insertions, this
from pygments.token import Text, Comment, Operator, Keyword, Name, String, \
    Number, Punctuation, Generic, Other, Error

#PythonLexer.EXTRA_CLASSNAMES = set(('AddSystem', 'AddObject', 'AddNode', 'AddMarker', 'AddLoad', 'AddSensor'))
PythonLexer.EXTRA_CLASSNAMES = set(listClassNames+listItemNames+listPyClassNames)
PythonLexer.EXTRA_FUNCTIONNAMES = set(listFunctionNames+listPyFunctionNames)

def ProcessTokens(self,text):
        for index, token, value in RegexLexer.get_tokens_unprocessed(self, text):
            if token is Name and value in self.EXTRA_CLASSNAMES:
                yield index, Name.Class, value   
            elif token is Name and value in self.EXTRA_FUNCTIONNAMES:
                yield index, Name.Function, value   
                #yield index, Keyword.Pseudo, value 
                #yield index, Operator.Word, value   
                #yield index, Name, value   
                #yield index, Name.Function, value   
            else:
                yield index, token, value

#monkey patch this function ...
PythonLexer.get_tokens_unprocessed = ProcessTokens
#style: see https://pygments.org/styles/
pygments_style = 'colorful' #colorful, native, vs, 

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

numfig = True #uses numbers for figures, see https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-numfig

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

templates_path = ['_templates']
#everything that is not documentation; before the flatten a single 'main/*' covered src,
#include, libs, obj and pythonDev, so each of them has to be named individually now
#NOTE README.rst is NOT excluded: it is a document of the
#documentation itself (the first page of the user manual) as well as the GitHub and PyPI
#landing page. One file, three places - and its image paths stay relative to the repository
#root, which is what GitHub and PyPI need.
exclude_patterns = ['rotorAnsys.rst',
                    'src/*','msvc/*','include/*','libs/*','python/*',
                    'tools/generators/generated/*',   #generated RST fragments, not documents
                    'docs/generated/notebooks/snippets/*', #examples the manual pages include (#2831)
                    '_build/*','build/*','dist/*','tmp/*','.pytest_cache/*',
                    'docs/generated/README.md',       #what the directory is, for humans in git
                    #Markdown that is NOT documentation. Sphinx reads .md
                    #since myst_parser was added, and everything it can read must either be in a
                    #toctree or excluded - a page in neither fails the strict build (step R7.1.2).
                    'docs/revision/*',   #the revision plan, log and info: a working record
                    'CLAUDE.md',         #the working contract for Claude Code sessions
                    '.github/*',         #issue and pull request templates: GitHub reads them,
                                         #Sphinx must not
                    #maintainer notes: they stay in the repository and are linked from the
                    #how-to section of the documentation, but they are not pages of the manual
                    #(#2585)
                    'docs/howTo/buildQuirks.md',
                    'docs/howTo/convertVideosFfmpeg.md',
                    'docs/howTo/gccVsMsvcTraps.md',
                    'docs/howTo/matplotlibExamples.md',
                    'docs/howTo/visualStudio2022.md',
                    #the mouse and keyboard tables, generated from
                    #python/exudyn/misc/keyBindings.py and INCLUDED by docs/manual/GUI.md;
                    #as documents of their own they would be orphans and fail the strict
                    #build (#2591)
                    'docs/generated/mouseBindings.md', 'docs/generated/keyBindings.md',
                    'docs/demo/*', 'docs/userTools/*', 'docs/verification/*']

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE PDF (#2586). "sphinx -M latexpdf . _buildpdf -t pdf" sets the tag,
#and only then is pdfIndex.md a document: it is the root of the PDF, its landing page and the
#only place the order of the PDF is written down. The html build never sees it, and this branch is
#the only difference between the two builds.
#What the PDF leaves out and why: README.rst (ten badge images fetched from the web as SVG, and
#three animated GIFs - none of which LaTeX can include; pdfIndex.md replaces it), and the 289
#pages of example and test model SOURCE LISTINGS, which are 517 of the ~1400 pages and are what a
#reader goes to github for. Nothing points at them internally: the item pages link the examples by
#their github URL (tools/generators/generatorPaths.py, githubSourceURL), so excluding them costs
#no reference. The issue tracker log stays IN, deliberately - it is a few hundred KB of why, and
#it is what makes the PDF worth searching (maintainer, 2026-09-22).
if tags.has('pdf'):                                                             # noqa: F821
    master_doc = 'pdfIndex'
    exclude_patterns += ['index.md', 'README.rst',
                         'docs/generated/examples/*', 'docs/generated/testModels/*']
else:
    exclude_patterns += ['pdfIndex.md']

#for google search index file, placed into root folder
html_extra_path = ['docs/extraHtml/googleeeca4e2177bc5628.html']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = "furo"
html_theme = "sphinx_rtd_theme"
#html_theme = 'classic'
#html_theme = "pydata_sphinx_theme"

#html_static_path = ["_static"]

#only works on readthedocs.io :
extensions = [
   'sphinx_search.extension', #pip install readthedocs-sphinx-search
   'sphinx_copybutton',
   'myst_parser',             #Markdown sources (#2546); the migration of
                              #step R7.1 converts the .tex chapters into this format
]

#a .md file is a document, a .rst file is a document; nothing else changes
source_suffix = {'.rst': 'restructuredtext', '.md': 'markdown'}

#a heading gets an anchor, so that [text](OTHER.md#a-heading) works between Markdown files - which
#is how the developer documentation already cross-references itself
myst_heading_anchors = 3

#WITHOUT dollarmath, MyST treats $...$ as ordinary text: the page then shows the LaTeX source
#and MathJax is not even loaded on it (measured on the first converted chapter, step R7.1.5).
#amsmath is for the align/aligned environments the chapters use. The macros themselves
#(\qv, \LU, \mr, ...) are resolved by MathJax through mathjax3_config below, which is why a
#converted chapter carries its math over verbatim - and also why the .md does NOT render in a
#plain Markdown preview such as the one of VS Code, which knows none of this project's macros.
myst_enable_extensions = ['dollarmath', 'amsmath']
#html_theme_path = ["_themes", ]

#for custom layout:
templates_path = ["_templates"]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rtd:
if html_theme == "sphinx_rtd_theme":
    html_static_path = ['docs/_static']
    html_css_files = ['custom.css']
    html_theme_options = {
    'prev_next_buttons_location': 'bottom', #bottom, top, both
    'style_external_links': False,
    #'vcs_pageview_mode': '',
    #'style_nav_header_background': 'white',
    # Toc options
    # 'collapse_navigation': True,
    # 'sticky_navigation': True,
    'navigation_depth': 3,
    # 'includehidden': True,
    'titles_only': False,
    }

#furo:
if html_theme == "furo":
    html_theme_options = {
        #"top_of_page_button": "edit",
        "navigation_with_keys": True,
        # "light_css_variables": {
            ##"font-stack": "Segoe UI",
            ##"font-stack--monospace": "Courier, monospace",
            # "font-size--normal": "100%",
            # "font-size--small": "87.5%",
            # "font-size--small--2": "81.25%",
            # "font-size--small--3": "75%",
            # "font-size--small--4": "62.5%",
            # "sidebar-caption-font-size": "100%",
            # "sidebar-item-font-size": "100%",
            # "api-font-size": "30%",
            # "admonition-font-size": "0.6125%",
            # "admonition-title-font-size": "0.6125%",
        # },
    }

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#this does some magic and will add macros for mathjax (Default in sphinx for math: / latex formulas)
#https://github.com/sphinx-doc/sphinx/issues/8195
#https://docs.mathjax.org/en/latest/input/tex/extensions/configmacros.html
packages: {'[+]': ['noerrors']}
mathjax3_config = {  
  #needed for color?
  #loader: {load: ['[tex]/color']},
  #tex: {packages: {'[+]': ['color']}}
    'loader': {
        'load': ['[tex]/mathtools']
    },
    'tex': {                        
        'packages': {#these packages are loaded [-] unloads 
            '[+]': ['mathtools']
        },
        'macros': { #write defs without '\' at beginning; use [,n] with n arguments
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the rest of the document's macros (#2549).
#Until the chapters became Markdown, the .tex -> .rst converter EXPANDED these
#(\qv was written into the .rst as \mathbf{q}), so MathJax never saw them and conf.py
#carried only the ones that survived expansion. The Markdown keeps the macro, so every
#macro a chapter uses has to be declared here - and here is now the ONLY place they live,
#because docincludes.sty and theDoc.tex die with the LaTeX build (info document D8).
            'ANCFdk': r'{\LU{0}{\dv_k}}',
            'ANCFdkO': r'{\LU{0}{\dv_{0,k}}}',
            'ANCFdkOtp': r'{\LU{0}{\dv_{0,k}\tp}}',
            'ANCFdkt': r'{\LU{0}{\dot\dv_k}}',
            'Am': r'{\mathbf{A}}',
            'Dm': r'{\mathbf{D}}',
            'Em': r'{\mathbf{E}}',
            'Gm': r'{\mathbf{G}}',
            'Im': r'{\mathbf{I}}',
            'ImThree': r'{\mathbf{I}_{3 \times 3}}',
            'Jm': r'{\mathbf{J}}',
            'Km': r'{\mathbf{K}}',
            'Mm': r'{\mathbf{M}}',
            'Pm': r'{\mathbf{P}}',
            'Rm': r'{\mathbf{R}}',
            'Sm': r'{\mathbf{S}}',
            'Tm': r'{\mathbf{T}}',
            'av': r'{\mathbf{a}}',
            'bv': r'{\mathbf{b}}',
            'cv': r'{\mathbf{c}}',
            'diffANCF': [r'{\frac{\partial #1}{\partial \qv_{ANCF}}}', 1],
            'diffANCFdk': r'{\diffANCFmI{\ANCFdk}}',
            'diffANCFmI': [r'{\frac{\partial #1}{\partial \qv_{ANCF,m1}}}', 1],
            'diffANCFmIt': [r'{\frac{\partial #1}{\partial \dot \qv_{ANCF,m1}}}', 1],
            'diffANCFt': [r'{\frac{\partial #1}{\partial \dot \qv_{ANCF}}}', 1],
            'diffmI': [r'{\frac{\partial #1}{\partial \qv_{m1}}}', 1],
            'diffmIt': [r'{\frac{\partial #1}{\partial \dot \qv_{m1}}}', 1],
            'diffmOI': [r'{\frac{\partial #1}{\partial \qv_{m0,m1}}}', 1],
            'diffmOIt': [r'{\frac{\partial #1}{\partial \dot \qv_{m0,m1}}}', 1],
            'diffmOt': [r'{\frac{\partial #1}{\partial \dot \qv_{m0}}}', 1],
            'dv': r'{\mathbf{d}}',
            'ev': r'{\mathbf{e}}',
            'fv': r'{\mathbf{f}}',
            'gv': r'{\mathbf{g}}',
            'hv': r'{\mathbf{h}}',
            'nv': r'{\mathbf{n}}',
            'pv': r'{\mathbf{p}}',
            'qv': r'{\mathbf{q}}',
            'rv': r'{\mathbf{r}}',
            'sv': r'{\mathbf{s}}',
            'tv': r'{\mathbf{t}}',
            'uv': r'{\mathbf{u}}',
            'vv': r'{\mathbf{v}}',
            'xv': r'{\mathbf{x}}',
            'yv': r'{\mathbf{y}}',
            'zv': r'{\mathbf{z}}',

            'vspace': [r'{}',1], #does not work with mathjax
#misc
            'ra': r'{\rightarrow}',
            'Rcal': r'{\mathbb{R}}',
            'Ccal': r'{\mathbb{C}}',
            'Ncal': r'{\mathbb{N}}',
            
#rotation, sin, cos
            'Rot': r'{\mathbf{A}}',
            'dd': r'{\mathrm{d}}',
            'ps': r'{p_\mathrm{s}}', #Euler parameters scalar

            'co': r'{\mathrm{c}}',
            'si': r'{\mathrm{s}}',

            'tp': r'{^\mathrm{T}}', #transpose

            'diag': r'{\mathrm{diag}}', #transpose
            'vec': r'{\mathrm{vec}}', #transpose

            'Null': r'{\mathbf{0}}',
#greek
            'varepsilonDot': r'{\boldsymbol{\varepsilon}}',
            'talpha': r'{\boldsymbol{\alpha}}',
            'tbeta': r'{\boldsymbol{\beta}}',
            'tgamma': r'{\boldsymbol{\gamma}}',
            'tchi': r'{\boldsymbol{\chi}}',
            'tdelta': r'{\boldsymbol{\delta}}',
            'teps': r'{\boldsymbol{\varepsilon}}',
            'tepsDot': r'{\boldsymbol{\dot \varepsilon}}',
            'teta': r'{\boldsymbol{\eta}}',
            'tkappa': r'{\boldsymbol{\kappa}}',
            'tkappaDot': r'{\boldsymbol{\dot \kappa}}',
            'tphi': r'{\boldsymbol{\phi}}',
            'boldVarPhi': r'{\boldsymbol{\varphi}}',
            'tPhi': r'{\boldsymbol{\Phi}}',
            'ttheta': r'{\boldsymbol{\theta}}',
            'tTheta': r'{\boldsymbol{\Theta}}',
            'tlambda': r'{\boldsymbol{\lambda}}',
            'tnu': r'{\boldsymbol{\nu}}',
            'tmu': r'{\boldsymbol{\mu}}',
            'tpsi': r'{\boldsymbol{\psi}}',
            'tPsi': r'{\boldsymbol{\Psi}}',
            'ttau': r'{\boldsymbol{\tau}}',
            'tsigma': r'{\boldsymbol{\sigma}}',
            'txi': r'{\boldsymbol{\xi}}',
            'tzeta': r'{\boldsymbol{\zeta}}',
            'tomega': r'{\boldsymbol{\omega}}',
            'tOmega': r'{\boldsymbol{\Omega}}',

            'vareps': r'{\varepsilon}',

#because \ov is replaced ..
            'myoverline': [r'\overline{#1}',1],

#vectors/matrices
            #'LU': [r'{\,^{#1}}',1],
            'pluseq': r'\mathrel{+}=',
            'LU': [r'{\prescript{#1}{}{#2}\,}',2],
            'LUX': [r'{\prescript{#1}{}{#2}#3\,}',3],
            'LUR': [r'{\prescript{#1}{}{#2}_{#3}\,}',3],
            'LURU': [r'{\prescript{#1}{}{#2}_{#3}^{#4}\,}',4],
            'LLdot': [r'{\prescript{}{#1}{\dot{#2}}_{#3}\,}',3],

            'vr': [r'{\left[ \begin{array}{c} { #1}\vspace{0.04cm} \\ { #2}\vspace{0.04cm} \\ { #3} \end{array} \right]}', 3], 
            'mr': [r'{\left[ \begin{array}{ccc} #1 & #2 & #3 \vspace{0.04cm}\\ #4 & #5 & #6 \vspace{0.04cm}\\ #7 & #8 & #9  \end{array} \right]}',9],
            'vp': [r'{\left[ \begin{array}{c} { #1} \vspace{0.04cm}\\ { #2} \end{array} \right]}', 2],
            'mp': [r'{\left[ \begin{array}{cc} #1 & #2 \vspace{0.04cm}\\ #3 & #4 \end{array} \right]}', 4],

            #mfour does not work ("misplaced &"):
            'mfour': [r'{\left[ \begin{array}{cccc} { #1} \\ { #2} \\ { #3} \\ { #4} \end{array} \right]}', 4], 
            'vfour': [r'{\left[ \begin{array}{c} { #1} \\ { #2} \\ { #3} \\ { #4} \end{array} \right]}', 4], 

            'vrRow': [r'{[#1,\, #2,\, #3]}', 3], 
            'vsix': [r'{\left[ \begin{array}{c} { #1} \\ { #2} \\ { #3} \\ { #4} \\ { #5} \\ { #6} \end{array} \right]}', 6], 
            'vsixb': [r'{\begin{array}{c} { #1} \\ { #2} \\ { #3} \\ { #4} \\ { #5} \\ { #6} \end{array} }', 6], 
            'vsixs': [r'{ \begin{array}{c} { #1} \\ { #2} \\ { #3} \\ { #4} \\ { #5} \\ { #6} \end{array} }', 6], 

#for system equations marking components
            'SO': r'{q}',
            'FO': r'{y}',
            'AE': r'{\lambda}',
            'SYS': r'{s}',

            'SON': r'{$2^\mathrm{nd}$ order differential equations}',
            'FON': r'{$1^\mathrm{st}$ order differential equations}',
            'AEN': r'{algebraic equations}',
            'Bm': r'{\mathbf{B}}',
            'Cm': r'{\mathbf{C}}',
            'Fm': r'{\mathbf{F}}',
            'SYSN': r'{system equations}',

#configurations subscripts
            'cIni': r'{_\mathrm{ini}}', #initial
            'cRef': r'{_\mathrm{ref}}', #reference
            'cCur': r'{_\mathrm{cur}}', #current
            'cVis': r'{_\mathrm{vis}}', #visualization
            'cSOS': r'{_\mathrm{start\;of\;step}}',
            'cConfig': r'{_\mathrm{config}}', #any configuration

#++++++++++++++++++++++++++++++++++++++++++
#special vectors
            'pLoc': r'{\mathbf{b}}',
            'pLocB': r'{\,^{b}{\mathbf{v}}}',
            'pRef': r'{\mathbf{r}}',
            'pRefG': r'{\,^{0}{\mathbf{r}}}',
            #'ImThree': r'{\mathbf{I}_{3 \times 3}}',#Im is replaced, so must ImThree
            #'ImTwo': r'{\mathbf{I}_{2 \times 2}}',

#++++++++++++++++++++++++++++++++++++++++++
#for FFRF:
            'indf': r'{_\mathrm{f}}',
            'indt': r'{_\mathrm{t}}',
            'indr': r'{_\mathrm{r}}',
            'indtt': r'{_\mathrm{tt}}',
            'indrr': r'{_\mathrm{rr}}',
            'indff': r'{_\mathrm{ff}}',
            'indtf': r'{_\mathrm{tf}}',
            'indrf': r'{_\mathrm{rf}}',
            'indtr': r'{_\mathrm{tr}}',

            'omegaBDtilde': r'{\LU{b}{\tilde \tomega_\mathrm{bd}}}',
#for FFRFreducedOrder:
            'indrigid': r'{_\mathrm{rigid}}',
            'indred': r'{_\mathrm{red}}',
            'induser': r'{_\mathrm{user}}',
            'indu': r'{_\mathrm{u}}',
#theory:
            'termA': [r'{\color{blue}{#1}}',1],
            'termB': [r'{\color{red}{#1}}',1],
            'termC': [r'{\color{green}{#1}}',1],
#solver:
            #braces around the argument: MathJax reads \ddot \mathbf{q} as intended, LaTeX reads
            #it as \ddot{\mathbf} and stops
            'acc': r'{\ddot{\mathbf{q}}}',
            'GA': r'{G\alpha}',
            'Hm': r'{\mathbf{H}}',
            'ImTwo': r'{\mathbf{I}_{2 \times 2}}',
            'Lm': r'{\mathbf{L}}',
            'Qm': r'{\mathbf{Q}}',
            'Vm': r'{\mathbf{V}}',
            'Wm': r'{\mathbf{W}}',
            'Xm': r'{\mathbf{X}}',
            'Ym': r'{\mathbf{Y}}',
            'aalg': r'{\mathbf{a}}',
            'iv': r'{\mathbf{i}}',
            'jv': r'{\mathbf{j}}',
            'kv': r'{\mathbf{k}}',
            'lv': r'{\mathbf{l}}',
            'mv': r'{\mathbf{m}}',
            'ov': r'{\mathbf{o}}',
            'vel': r'{\mathbf{v}}',
            'wv': r'{\mathbf{w}}',

            }
        }
    }

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE LATEX BUILD (#2586)
#Everything below is read by "sphinx -M latexpdf", which "exudev docs --pdf" runs, and by nothing
#else; the html build ignores it.
#
#THE MATH IS THE POINT. The chapters are written with this project's own macros - $\qv\cConfig$,
#$\LU{0b}{\Rot}$ - and the dict above is the ONE place they are declared (revision2026 step
#R7.1.5, #2549; tools/checkMathMacros.py is the gate that keeps it complete). MathJax reads that
#dict in the browser; LaTeX knows none of it, so the same dict is turned into the preamble here
#rather than written out a second time. \qv alone occurs 544 times in the pages of the PDF and
#\LU 2628 times, so an undeclared macro is not a cosmetic problem.
#FIVE of the names are commands LaTeX already has, which MathJax does not have to care about.
#They were found by asking LaTeX itself (\ifdefined over all of them), and each one is a decision
#that cannot be automated - so both lists are written out here, and everything else is emitted
#with \newcommand, which FAILS LOUDLY if a macro added later collides with something.
latexKeepsItsOwn = ['vspace']       #\vspace is layout: MathJax has a no-op for it (see above),
                                    #LaTeX does the real thing, and Sphinx's own output uses it
latexRedefined = ['AE', 'Im', 'mp', 'vec']
                                    #here the project's meaning has to win: AE is the Lagrange
                                    #multiplier and not the ligature, Im the identity matrix and
                                    #not the imaginary part, mp a 2x2 matrix and not the minus-
                                    #plus sign, vec the vec() operator and not the arrow accent
latex_engine = 'xelatex'            #the pages hold box-drawing characters and arrows in their
                                    #directory trees and tables; pdflatex has no glyph for them

latexMacroDefinitions = []
for (macroName, macroDefinition) in mathjax3_config['tex']['macros'].items():
    if macroName in latexKeepsItsOwn:
        continue
    if isinstance(macroDefinition, list):   #['body', numberOfArguments], as MathJax wants it
        (macroBody, argumentCount) = (macroDefinition[0], macroDefinition[1])
    else:
        (macroBody, argumentCount) = (macroDefinition, 0)
    latexMacroDefinitions.append(('\\renewcommand' if macroName in latexRedefined
                                  else '\\newcommand') + '{\\' + macroName + '}'
                                 + ('[' + str(argumentCount) + ']' if argumentCount else '')
                                 + '{' + macroBody + '}')

latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
    #mathtools for \prescript (the \LU family), xcolor for the \termA/B/C of the theory chapter
    #sphinxclassnotebookorigin: the notebook an example comes from, small and close under its code cell, the
    #container of class notebookorigin that tools/generators/autoGenerateHelper.py writes
    'preamble': ('\\usepackage{amssymb}\n\\usepackage{mathtools}\n\\usepackage{xcolor}\n'
                 + '\\newenvironment{sphinxclassnotebookorigin}{\\par\\vspace{-0.6\\baselineskip}'
                 + '\\footnotesize\\color{darkgray}}{\\par\\smallskip}\n'
                 + '%the math macros of conf.py, written by conf.py itself:\n'
                 + '\n'.join(latexMacroDefinitions) + '\n'),
    }

latex_documents = [('pdfIndex', 'exudynDocumentation.tex',
                    'Exudyn \u2014 Documentation', 'Johannes Gerstmayr', 'manual')]

#the flow charts are TikZ in docs/figures/tikz/, compiled by tools/buildFigures.py into a PDF (taken by
#the LaTeX build) and an SVG (taken by html) beside the other figures; nothing to configure here (#2812)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#CITATIONS (#2550)
#The chapters cite in running text - "see Zwoelfer and Gerstmayr [ZwoelferGerstmayr2021]" - which
#the LaTeX build resolved and nothing resolved after it: the keys were printed and pointed
#nowhere. docs/generated/references.md gives every entry of the bibliography a target, and the
#hook below appends one Markdown link definition per key to every Markdown document Sphinx reads.
#CommonMark then resolves [Key] as a shortcut reference link, and it does so ONLY where the text
#is text: a key inside a code fence or inline code stays what it is, because the Markdown parser
#decides, not a regular expression of ours. A definition that no page uses produces no output.
#The source files on disk are NOT touched; this happens to the text Sphinx has read.
citationDefinitions = "".join(["\n[" + key + "]: #ref-" + key.lower()
                               for key in citationKeys])


def AppendCitationDefinitions(app, docname, source):
    if docname == "docs/generated/references":   #it writes the targets; it must not link to them
        return
    if str(app.env.doc2path(docname)).endswith(".md"):
        source[0] = source[0] + "\n" + citationDefinitions + "\n"


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#LINE BREAKS INSIDE TABLE CELLS, FOR LATEX ONLY (#2586)
#The name cell of a settings table stacks the access paths of one item - the short name and the
#full visualizationSettings.general.xxx path - and the emitters separate them with a raw <br>
#(tools/generators/autoGenerateHelper.py). There are about 470 of them in SimulationSettings.md
#and VisualizationSettings.md. Raw html is html: the LaTeX writer drops it and warns, and the two
#paths would run together into one unreadable word in the PDF. Rather than change what the
#emitters write - the html is right, and a Markdown table cell cannot hold a real line break -
#the raw node is translated here, for the latex builder only.
def BreakToNewline(app, doctree, docname):
    if app.builder.format != "latex":
        return
    from docutils import nodes                    # noqa: PLC0415 - only the latex build needs it
    for node in list(doctree.findall(nodes.raw)):
        if node.get("format") == "html" and node.astext().strip().rstrip("/>").strip("<") == "br":
            node.replace_self(nodes.raw("", "\\newline ", format="latex"))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#TABLES IN THE PDF (#2853)
#Sphinx sets a table of up to 30 rows with tabulary, which cannot break across pages - a longer one ran into the
#footer - and which sizes its columns by a guess that formulas spoil: a column of $\mr{...}$ matrices squeezed the
#description of the notation tables to a single word per line. So, for the latex builder only: every table that
#stands in a section is a longtable, and a table without a tabularcolumns of its own gets column widths from its
#content - the typical length of its cells (the 80th percentile, so that one long default does not widen the
#column), bounded, so that a short column stays narrow and a long text gets the room.
def TableWidths(app, doctree, docname):
    if app.builder.format != "latex":
        return
    from docutils import nodes                    # noqa: PLC0415 - only the latex build needs it
    from sphinx import addnodes                   # noqa: PLC0415
    for table in list(doctree.findall(nodes.table)):
        parent = table.parent
        inSection = True                          #a longtable cannot stand in a box, a list or another table
        while parent is not None and not isinstance(parent, nodes.document):
            if not isinstance(parent, nodes.section):
                inSection = False
                break
            parent = parent.parent
        if inSection:
            table["classes"].append("longtable")
        index = table.parent.index(table)
        if index > 0 and isinstance(table.parent[index - 1], addnodes.tabular_col_spec):
            continue                              #the widths are given
        tgroup = table.next_node(nodes.tgroup)
        colspecs = [child for child in tgroup.children if isinstance(child, nodes.colspec)]
        lengths = [[] for _ in colspecs]
        for row in tgroup.findall(nodes.row):
            entries = [child for child in row.children if isinstance(child, nodes.entry)]
            if len(entries) != len(colspecs):     #a cell that spans columns says nothing about one column
                continue
            for (column, entry) in enumerate(entries):
                lengths[column].append(len(entry.astext()))
        headerWords = [[] for _ in colspecs]      #a word of the heading is not broken
        for row in tgroup.findall(nodes.row):
            if isinstance(row.parent, nodes.thead):
                entries = [child for child in row.children if isinstance(child, nodes.entry)]
                if len(entries) == len(colspecs):
                    for (column, entry) in enumerate(entries):
                        headerWords[column] += [len(word) for word in entry.astext().split()]
        for (colspec, columnLengths, words) in zip(colspecs, lengths, headerWords):
            columnLengths = sorted(columnLengths)
            typical = columnLengths[int(0.8 * (len(columnLengths) - 1))] if columnLengths else 10
            #+3 for the space between the columns, which a short column would otherwise lose to its neighbours
            colspec["colwidth"] = min(max(typical, max(words, default=4)), 50) + 3
        table["classes"].append("colwidths-given")


def setup(app):
    app.connect("source-read", AppendCitationDefinitions)
    app.connect("doctree-resolved", BreakToNewline)
    app.connect("doctree-resolved", TableWidths)
