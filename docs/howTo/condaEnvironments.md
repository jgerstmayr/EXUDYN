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

`venvExuP313` is the reference environment: it runs Exudyn, the tests and the documentation
build.

```bash
conda create -n venvExuP313 python=3.13 numpy scipy=1.15.2 matplotlib ipywidgets tqdm spyder-kernels=3.0 ipykernel psutil -y
conda activate venvExuP313
pip install exudyn ngsolve h5py sphinx readthedocs-sphinx-search sphinx-copybutton sphinx_rtd_theme
```

> **Pin scipy to 1.15.2.** scipy 1.18.0 slows the Exudyn test suite from ~22 s to over 10 minutes,
> apparently in the eigensolver path. Measured 2026-09-09 on the same machine and the same Exudyn
> 1.11.0; NGsolve makes no difference. scipy 1.15.2 pulls numpy 2.4.6, which is fine.

Test environments per Python version are named `venvP310` ... `venvP314` and carry an Exudyn build
for that version.

### Documentation toolchain only

If an existing environment only needs the docs tools added:

```bash
pip install sphinx readthedocs-sphinx-search sphinx-copybutton sphinx_rtd_theme
```

Build the HTML documentation from the repository root:

```bash
sphinx-build -b html . _build -E
```

### Advanced examples

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install stable-baselines3[extra]
pip install mpi4py
pip install numpy-stl numba
```

The `--index-url` above selects the CUDA 11.8 build of PyTorch; drop it for a CPU-only install.

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
