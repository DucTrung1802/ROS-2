from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize(
        [
            "primes.pyx",  # Cython code file with primes() function
            "primes_python_compiled.py",
            "test_fuzzy_logic_cython.pyx",
            "example_cython.pyx",
        ],  # Python code file with primes() function
    ),  # enables generation of the html annotation file
)
