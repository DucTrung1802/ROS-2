# setup.py
try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension

from Cython.Build import cythonize
import numpy

setup(
    ext_modules=cythonize(
        ["example_cython.pyx", "fuzzy_test_cypy.py", "fuzzy_test_cy.pyx"]
    ),
    include_dirs=[numpy.get_include()],
)
