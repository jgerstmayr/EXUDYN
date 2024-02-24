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
file='main/src/pythonGenerator/exudynVersion.py'
exec(open(file).read(), globals())

release = exudynVersionString

project = 'Exudyn'+release
copyright = '2023' #'2023, Johannes Gerstmayr'
author = 'Johannes Gerstmayr'
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#try to patch pygments Python style
#seems not to work:
# from confHelper import listClassNames, listFunctionNames

listClassNames=[]
listFunctionNames=[]
file='docs/RST/confHelper.py'
exec(open(file).read(), globals())
file='docs/RST/confHelperItems.py'
exec(open(file).read(), globals())
file='docs/RST/confHelperPyUtilities.py'
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
exclude_patterns = ['README.rst','rotorAnsys.rst','main/*','tools/*']

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
]
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
            'acc': r'{\ddot \mathbf{q}}',
            'GA': r'{G\alpha}',
            'aalg': r'{\mathbf{a}}',
            'vel': r'{\mathbf{v}}',

            }                       
        }                           
    }       






