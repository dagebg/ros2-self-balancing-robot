# Configuration file for the Sphinx documentation builder.
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import shutil
import subprocess
from pathlib import Path

import yaml

DOCS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = DOCS_DIR.parent
GENERATED_DIR = Path(__file__).resolve().parent / "_generated"

# -- Project information -----------------------------------------------------
project = "ROS2 Self-Balancing Robot"
copyright = "2026, David Geßner"
author = "David Geßner"
release = "0.1.0-dev"

# -- General configuration ---------------------------------------------------
extensions = [
    "breathe",  # C++ API docs via Doxygen XML
    "myst_parser",  # Markdown (MyST) pages
    "sphinx.ext.autodoc",  # Python docstring extraction
    "sphinx.ext.napoleon",  # Google/NumPy style docstrings
    "sphinx.ext.viewcode",  # "view source" links
    "sphinxcontrib.mermaid",  # Mermaid diagrams
]

source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# MyST: render ```mermaid fences as diagrams so the same Markdown also renders on GitHub.
myst_enable_extensions = ["colon_fence", "deflist", "dollarmath", "attrs_inline"]
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3

# -- Doxygen / Breathe -------------------------------------------------------
# Doxygen XML is generated on every build so the API pages never go stale.
# Set SKIP_DOXYGEN=1 to build prose-only docs on machines without doxygen.
DOXYGEN_XML = DOCS_DIR / "doxygen_output" / "xml"
if os.environ.get("SKIP_DOXYGEN") != "1":
    if shutil.which("doxygen") is None:
        raise RuntimeError("doxygen not found; install it or set SKIP_DOXYGEN=1")
    subprocess.run(["doxygen", "Doxyfile"], cwd=DOCS_DIR, check=True)

breathe_projects = {"robot": str(DOXYGEN_XML)}
breathe_default_project = "robot"
breathe_default_members = ("members", "undoc-members")

# -- Generated pages ---------------------------------------------------------
# The robot parameter table is rendered from config/robot_params.yaml so it never drifts.


def _render_params_table() -> str:
    params = yaml.safe_load((REPO_ROOT / "config" / "robot_params.yaml").read_text("utf-8"))
    rows = [
        "| Group | Parameter | Value | Unit | Status | Description |",
        "|---|---|---|---|---|---|",
    ]
    for group, entries in params.items():
        for name, entry in entries.items():
            status = "measured" if entry["measured"] else "**placeholder**"
            rows.append(
                f"| {group} | `{name}` | {entry['value']} | {entry['unit']} | {status} "
                f"| {entry['doc']} |"
            )
    return "\n".join(rows) + "\n"


GENERATED_DIR.mkdir(exist_ok=True)
(GENERATED_DIR / "robot_params_table.md").write_text(_render_params_table(), "utf-8")
exclude_patterns.append("_generated")

# -- HTML output -------------------------------------------------------------
html_theme = "furo"
html_static_path = ["_static"]
html_title = "ROS2 Self-Balancing Robot"
html_theme_options = {
    "source_repository": "https://github.com/dagebg/ros2-self-balancing-robot/",
    "source_branch": "main",
    "source_directory": "docs/source/",
}
