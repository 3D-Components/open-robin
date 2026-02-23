# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ROBIN'
copyright = '2025, 3D-Components AS & Mechatronics Innovation Lab'
author = '3D-Components AS & Mechatronics Innovation Lab'
release = '0.1.0'
version = '0.1.0'

# Project description
description = (
    'Reusable building blocks for robotic process intelligence (FIWARE + ROS 2)'
)

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.napoleon',
    'myst_parser',
    'sphinxcontrib.mermaid',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'

# -- Options for autodoc ----------------------------------------------------
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__',
}

# -- Options for intersphinx extension --------------------------------------
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'pandas': ('https://pandas.pydata.org/docs/', None),
    'ros2': ('https://docs.ros.org/en/humble/', None),
}

# -- Options for todo extension ---------------------------------------------
todo_include_todos = True

# -- Options for Napoleon extension -----------------------------------------
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True

# -- Options for MyST parser ------------------------------------------------
myst_enable_extensions = [
    'colon_fence',
    'deflist',
    'dollarmath',
    'fieldlist',
    'html_admonition',
    'html_image',
    'linkify',
    'replacements',
    'smartquotes',
    'strikethrough',
    'substitution',
    'tasklist',
]
myst_fence_as_directive = ['mermaid']

# -- Additional configuration -----------------------------------------------

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
html_css_files = ['custom.css']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# HTML theme options
html_theme_options = {
    'light_css_variables': {
        'color-brand-primary': '#1f6feb',
        'color-brand-content': '#1f6feb',
    },
    'dark_css_variables': {
        'color-brand-primary': '#6ea8fe',
        'color-brand-content': '#6ea8fe',
    },
}

html_title = f'{project} Documentation'

# Mermaid options
mermaid_version = '10.9.1'
mermaid_init_js = """
mermaid.initialize({
  startOnLoad: true,
  securityLevel: "loose",
  theme: "base",
  themeVariables: {
    fontFamily: "Inter, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif",
    fontSize: "15px",
    primaryColor: "#dbeafe",
    primaryBorderColor: "#1d4ed8",
    primaryTextColor: "#0f172a",
    secondaryColor: "#dcfce7",
    secondaryBorderColor: "#166534",
    tertiaryColor: "#fef3c7",
    tertiaryBorderColor: "#92400e",
    lineColor: "#475569",
    clusterBkg: "#f8fafc",
    clusterBorder: "#94a3b8",
    edgeLabelBackground: "#f8fafc"
  },
  flowchart: {
    curve: "basis",
    useMaxWidth: true,
    htmlLabels: true
  }
});
"""

# Output file base name for HTML help builder.
htmlhelp_basename = 'ROBINdoc'

# -- Options for LaTeX output ------------------------------------------------
latex_elements = {
    # Additional stuff for the LaTeX preamble.
    'preamble': r"""
        \usepackage{charter}
        \usepackage[defaultsans]{lato}
        \usepackage{inconsolata}
    """,
}

# Grouping the document tree into LaTeX files.
latex_documents = [
    (
        'index',
        'ROBIN.tex',
        'ROBIN Documentation',
        '3D-Components AS & Mechatronics Innovation Lab',
        'manual',
    ),
]

# -- Options for manual page output ------------------------------------------
man_pages = [('index', 'robin', 'ROBIN Documentation', [author], 1)]

# -- Options for Texinfo output ----------------------------------------------
texinfo_documents = [
    (
        'index',
        'ROBIN',
        'ROBIN Documentation',
        author,
        'ROBIN',
        'AI-driven human-robot interface system for robotic process intelligence.',
        'Miscellaneous',
    ),
]

# -- Options for Epub output -------------------------------------------------
epub_title = project
epub_author = author
epub_publisher = author
epub_copyright = copyright
