# setup.py
from setuptools import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize(["example_cython.pyx", "fuzzy_test.py"]))
