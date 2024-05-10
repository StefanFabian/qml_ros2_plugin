#!/usr/bin/env python3
# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

# Fix for Q_Property
from breathe.renderer.sphinxrenderer import DomainDirectiveFactory, CMacroObject
if "property" not in DomainDirectiveFactory.cpp_classes:
  DomainDirectiveFactory.cpp_classes["property"] = (CMacroObject, "macro")


# On ReadTheDocs, generate the Doxygen documentation
if os.environ.get('READTHEDOCS', None) == 'True':
  from subprocess import call
  call('doxygen')

# -- Project information -----------------------------------------------------

project = 'QML ROS2 Plugin'
copyright = '2021, Stefan Fabian'
author = 'Stefan Fabian'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
  "breathe",
  "sphinx.ext.autosectionlabel",
  "sphinx.ext.todo",
  "sphinx_rtd_theme"
]

html_theme = 'sphinx_rtd_theme'

todo_include_todos = True

# Breathe Configuration
breathe_projects = {"project": os.path.abspath("xml")}
breathe_default_project = "project"
breathe_default_sections = ('public*', 'property', 'func*')

# If true, current module path is prepended to all description unit titles
# such as functions
add_module_names = False

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
