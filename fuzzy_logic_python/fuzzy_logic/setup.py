# setup.py
try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension

from Cython.Build import cythonize

setup(
    ext_modules=cythonize(
        [
            "anfis.py",
            "clustering.py",
            "generic_fs.py",
            "mamdani_fs.py",
            "mf.py",
            # "rule_parser.py",
            "rules.py",
            "sugeno_fs.py",
            "terms.py",
            "transform.py",
            "type.py",
            "variables.py",
        ]
    )
)
