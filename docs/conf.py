# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import os
import sphinx_rtd_theme

# -- General configuration ------------------------------------------------

project = 'pybullet_planning'
year = '2020'
author = 'Caelan Garrett & Yijiang Huang'
copyright = '{0}, {1}'.format(year, author)
version = release = '0.5.0'

master_doc = 'index'
source_suffix = '.rst'
# templates_path = ['_templates', ]
exclude_patterns = ['_build', '**.ipynb_checkpoints', '_notebooks']

pygments_style = 'sphinx'
show_authors = True
add_module_names = True

extlinks = {
    'issue': ('https://github.com/yijiangh/pybullet_planning/issues/%s', '#'),
    'pr': ('https://github.com/yijiangh/pybullet_planning/pull/%s', 'PR #'),
}

# -- Extension configuration ------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.coverage',
    'sphinx.ext.doctest',
    'sphinx.ext.extlinks',
    'sphinx.ext.ifconfig',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx_rtd_theme',
]
if os.getenv('SPELLCHECK'):
    extensions += 'sphinxcontrib.spelling',
    spelling_show_suggestions = True
    spelling_lang = 'en_US'

# intersphinx options
# intersphinx_mapping = {'python': ('https://docs.python.org/', None),
#                        'roslibpy': ('http://roslibpy.readthedocs.org/en/latest/', None)}

# autodoc options
autodoc_default_options = {
    'member-order': 'bysource',
    'special-members': '__init__',
    'exclude-members': '__weakref__',
    'undoc-members': True,
    'private-members': True,
    'show-inheritance': True,
}

autodoc_member_order = 'alphabetical'

# autosummary options
autosummary_generate = True

# on_rtd is whether we are on readthedocs.org
# on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
html_theme = "sphinx_rtd_theme"
# options: https://sphinx-rtd-theme.readthedocs.io/en/latest/configuring.html

# napoleon options
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = True
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = False
napoleon_use_rtype = False

# -- Options for HTML output ----------------------------------------------

html_split_index = False
html_short_title = '%s-%s' % (project, version)
html_context = {}
html_static_path = ['_static']
html_last_updated_fmt = '%b %d, %Y'
html_copy_source = False
html_show_sourcelink = False
html_add_permalinks = ''
html_experimental_html5_writer = True
html_compact_lists = True
