# Conda environments for Exudyn

Notes on creating and maintaining conda environments for Exudyn — for development, for
documentation generation, and for running the examples.

- **Author**: Johannes Gerstmayr
- **Date**: 2026-09-09

For basic setup of Anaconda or Miniconda itself, see the Exudyn documentation.

## Dependency policy

**numpy is the only hard requirement** — it follows from the nature of the C++/Python coupling.
Everything else is optional: installing Exudyn pulls the minimum, and functions needing more raise
at the point of use. The lists below are therefore *conveniences for running examples*, not
install requirements.

| package | needed for |
|---|---|
| numpy | required, always |
| scipy | FEM, eigenvalue computations, sparse solvers — **pin to 1.15.2**, see note below |
| matplotlib | plotting in examples and `exudyn.plot` |
| ngsolve, h5py | FEM mesh import and `FEMinterface` |
| numpy-stl | STL import/export |
| numba | a few accelerated example kernels |
| torch, stable-baselines3 | reinforcement learning examples |
| mpi4py | parallel parameter-variation examples |
| tqdm, psutil | progress bars and process info in `exudyn.processing` |
| ipywidgets, ipykernel | Jupyter notebook usage |
| spyder-kernels | Spyder integration (version must match Spyder, see below) |

## The standard environment

`venvExuP313` is the reference environment: it runs Exudyn, the generators, the tests and the
documentation build. Its packages are not listed here but in `pyproject.toml`, as dependency
groups (PEP 735), so this recipe cannot fall behind:

```bash
conda create -n venvExuP313 python=3.13 -y
conda activate venvExuP313
cd EXUDYN_git                                 #the repository root, where pyproject.toml is
python -m pip install --upgrade pip           #dependency groups need pip >= 25.1
pip install --group dev                       #docs, lint, build and IDE tools, scipy pin, jinja2, griffe
pip wheel . -w dist --no-deps                 #build Exudyn
pip install --pre --find-links=dist "exudyn[tests]"   #the local wheel plus what TestModels/ needs
```

| group | contains | used by |
|---|---|---|
| `docs` | sphinx and its theme and extensions | `sphinx-build`, the CI `docs` jobs, readthedocs |
| `lint` | `pydoclint` (pinned; the baseline holds its messages) | CI `check_docstrings` |
| `build` | `setuptools`, `wheel`, `pybind11<3.0` | a direct `python setup.py bdist_wheel`, which has no build isolation |
| `ide` | `spyder-kernels`, `ipykernel`, `ipywidgets` | Spyder and Jupyter |
| `dev` | all of the above, plus `scipy==1.15.2`, `jinja2`, `griffe` | the development environment |

Groups are never published with the wheel, unlike the extras (`exudyn[tests]` etc.) below, which
users see. A single group can be installed alone, e.g. `pip install --group docs`.

> **Why `pybind11` in `build`?** It is declared in `build-system.requires`, so `pip wheel .` and
> `python -m build` install it themselves under build isolation. A direct
> `python setup.py bdist_wheel` does **not** use build isolation, so there it has to be present in
> the environment.

> **Why scipy 1.15.2?** scipy 1.18.0 slows the Exudyn test suite from ~22 s to over 10 minutes,
> apparently in the eigensolver path (measured 2026-09-09, revision2026 fact 19). The pin is in
> the `dev` group; `exudyn[tests]` itself does not pin scipy.

> **spyder-kernels** in the `ide` group is `3.*`, matching Spyder 6; for an older Spyder see the
> table below and install the matching version afterwards.

Test environments per Python version are named `venvP310` ... `venvP314` and carry an Exudyn build
for that version.

### Documentation toolchain only

If an existing environment only needs the docs tools added, from the repository root:

```bash
pip install --group docs
```

Build the HTML documentation from the repository root:

```bash
sphinx-build -b html . _build -E
```

### Optional packages via the Exudyn extras

The optional dependencies are declared in `pyproject.toml`, so they can be installed by name
instead of being listed by hand:

| command | installs |
|---|---|
| `pip install exudyn[tests]` | exactly what `TestModels/` needs: scipy, matplotlib, h5py, networkx, psutil, ngsolve |
| `pip install exudyn[common]` | the above plus what frequently used features need: numpy-stl (STL import), tqdm (optimization progress), ffmpeg-python (video export) |
| `pip install exudyn[all]` | the above plus everything else the package and the Examples refer to: pymeshlab, roboticstoolbox-python, spatialmath-python, numba, dispy, mpi4py |
| `pip install exudyn[rl]` | reinforcement learning: torch, stable-baselines3, gymnasium, gym, tensorboard |

`[rl]` is deliberately **not** part of `[all]`: torch is multi-GB and the CPU/CUDA choice is made
with an `--index-url`, which cannot be expressed in wheel metadata. For a CUDA build, install it
separately first:

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install stable-baselines3[extra]
```

The `--index-url` above selects the CUDA 11.8 build of PyTorch; drop it for a CPU-only install.

These lists are not maintained by hand. `python tools/checkExtras.py` scans every import in
`exudyn/`, `TestModels/` and `Examples/` and fails if one of them is installed by no extra, so the
table above cannot silently fall behind the code; CI runs it as the `check_extras` job.

## Spyder kernel versions

`spyder-kernels` must match the Spyder version, or the console fails to start:

| Anaconda / Spyder | spyder-kernels |
|---|---|
| Anaconda 2023-07 / Spyder 5.4 | `spyder-kernels=2.4` |
| Anaconda 2023-09 / Spyder 5.5, 5.5.1 | `spyder-kernels=2.5` |
| Anaconda 2025-06 / Spyder 6.0.7 | `spyder-kernels=3.0` |

## Useful conda commands

```bash
conda activate ENVNAME              # activate an environment
conda deactivate                    # leave it

conda env list                      # list all environments
conda list                          # list packages in the active environment

conda install numpy                 # install into the active environment
conda install -n ENVNAME numpy      # install into a named environment
conda update numpy                  # update in the active environment
conda remove --name ENVNAME numpy   # remove a package

conda remove --name ENVNAME --all -y   # delete an entire environment
conda update -n base -c defaults conda -y   # update conda itself
```

## manylinux wheels (auditwheel)

Now normally done with Docker; kept here for reference.

```bash
pip install auditwheel patchelf

auditwheel repair /mnt/c/DATA/cpp/Exudyn_git/main/dist/exudyn-1.10.19.dev1-cp314-cp314-linux_x86_64.whl
# produces e.g. exudyn-1.10.19.dev1-cp314-cp314-manylinux_2_34_x86_64.whl
```
