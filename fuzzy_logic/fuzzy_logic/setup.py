# setup.py
try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension

from Cython.Build import cythonize

setup(ext_modules=cythonize(["anfis.py", "mf.py", "terms.py", "variables.py", "mamdani_fs.py"]))
