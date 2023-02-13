# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Exudyn'
copyright = '2023' #'2023, Johannes Gerstmayr'
author = 'Johannes Gerstmayr'

#create exudynVersionString
file='main/src/pythonGenerator/exudynVersion.py'
exec(open(file).read(), globals())

release = exudynVersionString

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

templates_path = ['_templates']
exclude_patterns = ['README.rst','rotorAnsys.rst','main/*','tools/*','tools/*']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = "furo"
html_theme = "sphinx_rtd_theme"
#html_theme = 'classic'
#html_theme = "pydata_sphinx_theme"

#html_static_path = ["_static"]

#only works on readthedocs.io :
# extensions = [
   # 'sphinx_search.extension', #pip install readthedocs-sphinx-search
# ]
#html_theme_path = ["_themes", ]

#rtd:
if html_theme == "sphinx_rtd_theme":
    html_theme_options = {
    'prev_next_buttons_location': 'bottom', #bottom, top, both
    'style_external_links': False,
    'vcs_pageview_mode': '',
    #'style_nav_header_background': 'white',
    # Toc options
    # 'collapse_navigation': True,
    # 'sticky_navigation': True,
    'navigation_depth': 4,
    # 'includehidden': True,
    #'titles_only': False,
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


