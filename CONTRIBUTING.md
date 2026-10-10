# Contributing

This is a personal project, but it follows the workflow of a team project so that every change is
reviewable and reproducible.

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt -r docs/requirements.txt
.venv/bin/pre-commit install
```

The pre-commit hooks (formatting, linting, file checks) then run on every commit, and CI runs the
same hooks on every pull request. The full tool list is in the docs under *Development
environment*.

## Branches and pull requests

- `main` is always releasable; all work happens on short-lived branches merged through pull
  requests.
- Branch names use a type prefix: `feat/…`, `fix/…`, `refactor/…`, `docs/…`, `ci/…`, `chore/…`.
- A pull request contains code, tests and documentation for one change, and must pass CI.

## Commit messages

Commits follow [Conventional Commits](https://www.conventionalcommits.org/):

```text
<type>(<scope>): <summary in imperative mood>
```

- Types: `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `ci`, `build`, `chore`.
- Scopes: `firmware`, `protocol`, `core`, `ros2`, `sim`, `webapp`, `docs`, `ci`.
- Example: `feat(firmware): add PCNT quadrature encoder driver`.

## Documentation

Documentation lives in `docs/source/` (Sphinx with Markdown via MyST) and is part of every change:

- New modules get a page or a section, and public C++ APIs get Doxygen comments.
- Significant technical decisions get an ADR in `docs/source/development/adr/`.
- User-visible changes get an entry under **Unreleased** in `CHANGELOG.md`.

Build the docs locally (requires `doxygen`):

```bash
python3 -m venv .venv
.venv/bin/pip install -r docs/requirements.txt
make -C docs html SPHINXBUILD=../.venv/bin/sphinx-build SPHINXOPTS="-W --keep-going"
open docs/build/html/index.html
```

Warnings are treated as errors in CI.
