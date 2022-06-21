"""
Luferov Victor <lyferov@yandex.ru>

Fuzzy Terms
"""
import cython
from .mf import MembershipFunction

@cython.cclass
class Term:
    """
    Fuzzy term
    """

    def __init__(self, name: cython.char, mf: MembershipFunction):
        self.name: cython.char = name
        self.mf: MembershipFunction = mf
