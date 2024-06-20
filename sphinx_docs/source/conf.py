# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import sys
sys.path.append("/home/me/docproj/ext/breathe/")
project = 'iSAM2 SLAM'
copyright = '2024, Carnegie Mellon Racing'
author = 'Carnegie Mellon Racing'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx_rtd_theme',
    'breathe',
    'sphinx.ext.autosectionlabel'
]

templates_path = ['_templates']
exclude_patterns = []

breathe_projects = {"iSAM2 SLAM": "../../doxy_docs/xml/"}
breathe_default_project = "iSAM2 SLAM"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
