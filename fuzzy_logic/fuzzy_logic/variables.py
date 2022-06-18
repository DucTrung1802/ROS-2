"""
Luferov Victor <lyferov@yandex.ru>

Fuzzy Variable
    - Mamdani
    - Sugeno
"""

import cython
from abc import ABC, abstractmethod
from typing import List, Dict
from .terms import Term


class FuzzyVariable:

    def __init__(self, name: cython.char, min_value: cython.double = 0.0, max_value: cython.double = 1.0, *terms: Term):
        if min_value >= max_value:
            raise ValueError(f'{min_value} <= {max_value} is not True')
        self.name: cython.char = name
        self.min_value: cython.double = min_value
        self.max_value: cython.double = max_value
        self.terms: List[Term] = list(terms)

    def term_by_name(self, name: cython.char) -> Term:
        """
        Find term by name
        :param name: name of term
        :return: term
        """
        for term in self.terms:
            if term.name == name:
                return term

    @property
    def values(self):
        return self.terms


class SugenoFunction(ABC):
    """
    Sugeno function interfaces
    """
    @property
    @abstractmethod
    def name(self) -> cython.char:
        ...

    @abstractmethod
    def evaluate(self, inputs: Dict[FuzzyVariable, cython.double]) -> cython.double:
        ...


class LinearSugenoFunction(SugenoFunction):

    def __init__(self, name: str, coefficients: Dict[FuzzyVariable, cython.double], const: cython.double = .0):
        self.__name: cython.char = name
        self.coefficients: Dict[FuzzyVariable, float] = coefficients
        self.const: cython.double = const

    @property
    def name(self) -> cython.char:
        return self.__name

    def evaluate(self, inputs: Dict[FuzzyVariable, cython.double]) -> cython.double:
        """
        Calculate linear function
        :param inputs: Values
        :return: result of calculation
        """
        return self.const + sum([self.coefficients[variable] * value for variable, value in inputs.items()])


class SugenoVariable:
    """
    Sugeno variable
    """

    def __init__(self, name: cython.char, *functions: SugenoFunction):
        self.name: cython.char = name
        self.functions: List[SugenoFunction] = list(functions)

    def function_by_name(self, name: cython.char) -> SugenoFunction:
        for function in self.functions:
            if function.name == name:
                return function

    @property
    def values(self):
        return self.functions
