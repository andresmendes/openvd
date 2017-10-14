# H1 -- Top of Page Header
# ************************
# There should only be one of these per page and this will also -- when
# converting to pdf -- be used for the chapters.
#
# H2 -- Page Sections
# ===================
#
# H3 -- Subsection
# ----------------
# `Docs for this project <http://packages.python.org/an_example_pypi_project/>`_
# H4 -- Subsubsection
# +++++++++++++++++++

# https://pythonhosted.org/an_example_pypi_project/sphinx.html

from numpy import linspace
from control import ss

sys = ss("1. -2; 3. -4", "5.; 7", "6. 8", "9.")

tempo = linspace(0, 10, 11)

# x0 = array([[0], [0]])

# T, yout = step_response(sys, tempo, x0)
