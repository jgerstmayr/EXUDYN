#+++++++++++++++++++++++++++++++++++++++++++
#SPHINX
#information on installation of EXUDYN on WSL2
#WSL2 runs on Windows 10 
#author: Johannes Gerstmayr
#date: 2023-02-10
#+++++++++++++++++++++++++++++++++++++++++++
#to run Exudyn sphinx build, we need to install:

#installation, needed for exudyn docs compilation (2023-09):
pip install sphinx
pip install readthedocs-sphinx-search
pip install sphinx-copybutton
pip install sphinx_rtd_theme

#themes: readthedocs-sphinx-search only works on server (git pages), not locally on your pc opening file in browser
pip install furo
pip install sphinx-copybutton
#pip install readthedocs-sphinx-search #now included as a standard; only works on readthedocs

#pip install sphinx-tabs #?

#pip install pydata-sphinx-theme --pre
#pip install pydata-sphinx-theme==0.13.0rc3.dev0

#create example directory:
sphinx-quickstart

#+++++++++++++++++++++++++++++++++++++++++++
#to build the files of directory source into directory build (source needs a conf.py and index.rst file), run:
sphinx-build -b html source build

#use -E option to process all files, do not use cache (may not always update all files ...):
sphinx-build -b html source build -E

#For Exudyn, go into Exudyn_git root directory and run:
sphinx-build -b html . _build -E

#+++++++++++++++++++++++++++++++++++++++++++
#settings on github:
#add .yaml or .yml file in .github/workflows
#go to general Settings/Pages
#  Build and deployment: 
#    - Deploy from a branch
#    - SELECT: gh-pages  +  /root  + Save
#under Actions, you see:
#  Deplay Github Pages V2: this action comes from documentation.yaml
#  pages-build-deployment: this comes from github internal / gh-pages
#+++++++++++++++++++++++++++++++++++++++++++
FOR EXUDYN Github pages test build, run in EXUDYN_git dir:
sphinx-build -b html . _build

#+++++++++++++++++++++++++++++++++++++++++++
#github pages:
https://www.sphinx-doc.org/en/master/tutorial/deploying.html#id5


#The GitHub Pages workflow used to be copied here verbatim. Removed 2026-09-10: the copy had
#already drifted from the real file (it still named ubuntu-20.04, actions/checkout@v2 and
#peaceiris/actions-gh-pages@v3, while the live workflow uses ubuntu-latest, checkout@v3 and
#JamesIves/github-pages-deploy-action@v4). A stale copy of a config is worse than no copy.
#
#The workflow itself:            .github/workflows/documentation.yaml
#The GitLab docs job (during the GitHub freeze, builds with -W but does not deploy):
#                                .gitlab-ci.yml
