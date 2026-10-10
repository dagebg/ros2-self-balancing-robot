# Configuration file for the Sphinx documentation builder.
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import shutil
import subprocess
from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parent.parent

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

# -- HTML output -------------------------------------------------------------
html_theme = "furo"
html_static_path = ["_static"]
html_title = "ROS2 Self-Balancing Robot"
html_theme_options = {
    "source_repository": "https://github.com/dagebg/ros2-self-balancing-robot/",
    "source_branch": "main",
    "source_directory": "docs/source/",
}
