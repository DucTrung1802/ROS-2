"""
Luferov Victor <lyferov@yandex.ru>

Membership functions
"""
import cython
from typing import List, Tuple
from abc import ABC, abstractmethod
import numpy as np
from .type import MfCompositionType


class MembershipFunction(ABC):
    """
    Abstract class of MF
    """

    @property
    @abstractmethod
    def sup(self) -> float:
        ...

    @abstractmethod
    def get_value(self, x: float) -> float:
        ...


class NormalMF(MembershipFunction):
    def __init__(self, b: cython.double, sigma: cython.double):
        self.b: cython.double = b
        self.sigma: cython.double = sigma

    def get_value(self, x: cython.double) -> cython.double:
        return np.exp(-((x - self.b) ** 2) / (2 * self.sigma ** 2))

    @property
    def sup(self) -> cython.double:
        return 1.0


class ConstantMF(MembershipFunction):
    """
    Singletone MF
    """

    def __init__(self, value: cython.double):
        if not 0.0 <= value <= 1:
            raise ValueError(f"0.0 <= {value} <= 1.0 is not True")
        self.value: cython.double = value

    def get_value(self, x: cython.double) -> cython.double:
        return self.value

    @property
    def sup(self) -> cython.double:
        return 1.0


class PointsMF(MembershipFunction):
    """
    Points MF
    """

    def __init__(self, points: List[Tuple[cython.double, cython.double]]):
        self.points: List[Tuple[cython.double, cython.double]] = points

    def get_value(self, x: cython.double) -> cython.double:
        pass


class TriangularMF(MembershipFunction):
    """
    Triangular MF
    """

    def __init__(self, x1: cython.double, x2: cython.double, x3: cython.double):
        if not (x1 <= x2 <= x3):
            raise ValueError(f"{x1} <= {x2} <= {x3} is not True")
        self.x1: cython.double = x1
        self.x2: cython.double = x2
        self.x3: cython.double = x3

    def get_value(self, x: cython.double) -> cython.double:
        """
        Get value of mf
        :param x: point of x
        :return: value of mf
        """
        if self.x1 == self.x2 == x or self.x2 == self.x3 == x or self.x2 == x:
            return 1.0
        elif self.x1 < x < self.x2:
            return x / (self.x2 - self.x1) - self.x1 / (self.x2 - self.x1)
        elif self.x2 < x < self.x3:
            return -x / (self.x3 - self.x2) + self.x3 / (self.x3 - self.x2)
        else:
            return 0.0

    def to_normal(self) -> NormalMF:
        return NormalMF(self.x2, (self.x3 - self.x1) / 5.0)

    @property
    def sup(self) -> cython.double:
        return 1.0


class TrapezoidMF(MembershipFunction):
    """
    Trapezoid MF
    """

    def __init__(
        self, x1: cython.double, x2: cython.double, x3: cython.double, x4: cython.double
    ):
        if not (x1 <= x2 <= x3 <= x4):
            raise ValueError(f"{x1} <= {x2} <= {x3} <= {x4} is not True")
        self.x1: cython.double = x1
        self.x2: cython.double = x2
        self.x3: cython.double = x3
        self.x4: cython.double = x4

    def get_value(self, x: cython.double) -> cython.double:
        """
        Get value of mf
        :param x: point of x
        :return: value of mf
        """
        if (
            self.x1 == self.x2 == x
            or self.x3 == self.x4 == x
            or self.x2 <= x <= self.x3
        ):
            return float(1)
        elif self.x1 < x < self.x2:
            return float(x / (self.x2 - self.x1) - self.x1 / (self.x2 - self.x1))
        elif self.x3 < x < self.x4:
            return float(-x / (self.x4 - self.x3) + self.x4 / (self.x4 - self.x3))
        else:
            return float(0)

    @property
    def sup(self) -> cython.double:
        return 1


class CompositeMF(MembershipFunction):
    """
    Composite  MF
    """

    def __init__(self, composite_type: MfCompositionType, *mfs: MembershipFunction):
        self.composite_type: MfCompositionType = composite_type
        self.mfs = mfs

    def __compose(self, x: List[cython.double]) -> cython.double:
        if self.composite_type == MfCompositionType.MAX:
            return np.max(x)
        elif self.composite_type == MfCompositionType.MIN:
            return np.min(x)
        elif self.composite_type == MfCompositionType.PROD:
            return float(np.prod(x))
        elif self.composite_type == MfCompositionType.SUM:
            return float(np.prod(x) - np.sum(x))
        else:
            raise ValueError(f"Type of composition {self.composite_type} is not found!")

    def get_value(self, x: cython.double) -> cython.double:
        return self.__compose([mf.get_value(x) for mf in self.mfs])

    @property
    def sup(self) -> cython.double:
        return 1.0
