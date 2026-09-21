# Building the documentation

The documentation is Sphinx, built from the repository root: `conf.py` and the hand-written
`index.md` are there, and the pages live in `docs/manual/` (the hand-written user manual),
`docs/generated/` (written by the emitters and the issue tracker — do not edit), `docs/dev/` and
`docs/howTo/`.

## The short way

```powershell
python -m exudev docs
```

That is the same build the CI runs: `sphinx-build -b html . _build -E -W --keep-going`, in the
environment the driver knows. **`-W` turns every warning into an error**, so a broken cross
reference, a document that is in no toctree or an image that is not there fails the build rather
than being printed and forgotten.

The result is `_build/index.html`; open it in a browser.

## What has to be installed

The documentation dependencies are a group in `pyproject.toml`, so they are installed by name and
not from a list that drifts:

```powershell
pip install --group docs
```

That group holds Sphinx itself, the `sphinx_rtd_theme`, `sphinx-copybutton`,
`readthedocs-sphinx-search` and `myst-parser` (Markdown). The search extension only works on a
server — on a local page it does nothing.

## Building by hand

If you want to run Sphinx yourself, from the repository root:

```powershell
sphinx-build -b html . _build          #incremental
sphinx-build -b html . _build -E       #ignore the cache: reads every file again
sphinx-build -b html . _build -E -W --keep-going    #what the gate does
```

`-E` matters more often than it should: a page whose *source* changed is rebuilt, but a page that
changed because something it includes changed is not always.

## Where it is published

- **readthedocs**: <https://exudyn.readthedocs.io>
- **GitHub Pages**: built by `.github/workflows/documentation.yaml` and deployed to the `gh-pages`
  branch (Settings → Pages → *Deploy from a branch*, `gh-pages` + `/root`). Under *Actions* two
  jobs appear: the one from that workflow, and GitHub's own `pages-build-deployment`.
- **GitLab** builds the documentation during the GitHub freeze (`.gitlab-ci.yml`) with `-W`, but
  deploys nothing.

The workflow file is deliberately **not** copied into this page: a copy of a configuration drifts
from the original and is then worse than no copy.

Sphinx's own tutorial on deploying is at
<https://www.sphinx-doc.org/en/master/tutorial/deploying.html>.
