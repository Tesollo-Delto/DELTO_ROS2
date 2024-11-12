# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
import os
import sys
# sys.path.insert(0, os.path.abspath('.'))   # Default path
# sys.path.insert(0, os.path.abspath('../..'))   # 필자의 path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

# delto_3f_driver 패키지 경로 추가
sys.path.insert(0, os.path.join(project_root, 'delto_3f_driver'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'delto'
copyright = '2024, hongcheol'
author = 'hongcheol'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# extensions = [sphinx.ext.autodoc]
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.napoleon',
              'sphinx.ext.viewcode']
templates_path = ['_templates']
exclude_patterns = ['build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['_static']
html_theme_path = ['_themes', ]
