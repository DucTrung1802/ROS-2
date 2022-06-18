"""
Luferov Victor <lyferov@yandex.ru>

Mamdani Fuzzy System
"""
import cython
from typing import List, Dict
from .generic_fs import GenericFuzzySystem
from .rules import FuzzyRule
from .variables import FuzzyVariable
from .rule_parser import RuleParser
from .mf import MembershipFunction, CompositeMF, ConstantMF
from .terms import Term
from .type import (
    AndMethod,
    OrMethod,
    ImplicationMethod,
    AggregationMethod,
    DefazzificationMethod,
    MfCompositionType,
)


class MamdaniFuzzySystem(GenericFuzzySystem):
    """ """

    def __init__(
        self,
        inp=None,
        out=None,
        am: AndMethod = AndMethod.MIN,
        om: OrMethod = OrMethod.MAX,
        im: ImplicationMethod = ImplicationMethod.MIN,
        ag: AggregationMethod = AggregationMethod.MAX,
        dm: DefazzificationMethod = DefazzificationMethod.CENTROID,
    ):
        """
        Конструктор создания нечеткой модели мамдани
        :param inp: входящие переменные
        :param out: выходные переменные
        :param am: метод и
        :param om: метод или
        :param im: метод нечеткой импликации
        :param ag: метод неечткого агрегирования
        :param dm: метод дефаззификации
        """
        self.out: List[FuzzyVariable] = out if out is not None else []
        self.implication_method: ImplicationMethod = im
        self.aggregation_method: AggregationMethod = ag
        self.def_method: DefazzificationMethod = dm
        super().__init__(inp if inp is not None else [], am, om)

    def output_by_name(self, name: cython.char) -> FuzzyVariable:
        """
        Ищем выходную переменную по имени
        :param name: имя переменной
        :return: переменная
        """
        for out in self.out:
            if out.name == name:
                return out
        raise Exception(f'Выходной переменной с именем "{name}" не найдено')

    def parse_rule(self, rule: cython.char) -> FuzzyRule:
        """
        Парсим правило из текста
        :param rule: правило в текстовом представлении
        :return: нечеткое правило
        """
        return RuleParser.parse(rule, self.inp, self.out)

    def calculate(
        self, input_values: Dict[FuzzyVariable, cython.double]
    ) -> Dict[FuzzyVariable, cython.double]:
        if len(self.rules) == 0:
            raise Exception("Должно быть как минимум одно правило")
        fi: Dict[FuzzyVariable, Dict[Term, cython.double]] = self.fuzzify(
            input_values
        )  # Шаг фаззификации
        conditions: Dict[FuzzyRule, cython.double] = self.evaluate_conditions(
            fi
        )  # Вычисляем состояния
        conclusions: Dict[FuzzyRule, MembershipFunction] = self.implicate(
            conditions
        )  # Вычисляем последствия
        fuzzy_result: Dict[FuzzyVariable, MembershipFunction] = self.aggregate(
            conclusions
        )  # Агрегация результатов
        result: Dict[FuzzyVariable, cython.double] = self.defuzzify(
            fuzzy_result
        )  # Дефаззафикация
        return result

    def implicate(
        self, conditions: Dict[FuzzyRule, cython.double]
    ) -> Dict[FuzzyRule, MembershipFunction]:
        """
        Функция импликации
        :param conditions: заключения
        :return:
        """

        def implicate_type(it: ImplicationMethod) -> MfCompositionType:
            if it == ImplicationMethod.MIN:
                return MfCompositionType.MIN
            elif it == ImplicationMethod.PROD:
                return MfCompositionType.PROD
            raise Exception(f"Тип композиции {it} не найден")

        return {
            rule: CompositeMF(
                implicate_type(self.implication_method),
                ConstantMF(value),
                rule.conclusion.term.mf,
            )
            for rule, value in conditions.items()
        }

    def aggregate(
        self, conclusions: Dict[FuzzyRule, MembershipFunction]
    ) -> Dict[FuzzyVariable, MembershipFunction]:
        """
        Процедура агрегации
        :param conclusions: заключения
        :return:
        """

        def composite_type(am: AggregationMethod) -> MfCompositionType:
            if am == AggregationMethod.MAX:
                return MfCompositionType.MAX
            elif am == AggregationMethod.SUM:
                return MfCompositionType.SUM
            raise Exception(f"Тип композиции {am} не найден")

        return {
            variable: CompositeMF(
                composite_type(self.aggregation_method),
                *[
                    mf
                    for rule, mf in conclusions.items()
                    if rule.conclusion.variable == variable
                ],
            )
            for variable in self.out
        }

    def defuzzify(
        self, fr: Dict[FuzzyVariable, MembershipFunction]
    ) -> Dict[FuzzyVariable, cython.double]:
        """
        Дефаззификация нечеткой переменной
        :param fr: fuzzyResult - результат нечеткой входной переменной
        :return: результат дефаззификации
        """
        return {
            variable: self.__defuzzify(mf, variable.min_value, variable.max_value)
            for variable, mf in fr.items()
        }

    def __defuzzify(
        self, mf: MembershipFunction, min_value: cython.double, max_value: cython.double
    ) -> cython.double:
        """
        Дефаззификация значения
        :param mf: лингвистический терм
        :param min_value: минимальное значение
        :param max_value: максимальное значение
        :return:
        """
        if self.def_method == DefazzificationMethod.CENTROID:
            i: cython.int
            k: cython.int = 101  # Шаг дефаззицикации
            step = (max_value - min_value) / k
            numerator: cython.double = 0
            denominator: cython.double = 0
            for i in range(k):
                pt_center: cython.double = min_value + step * i
                val_center: cython.double = mf.get_value(pt_center)
                val2_center: cython.double = pt_center * val_center
                numerator += val2_center
                denominator += val_center
            return round(numerator / denominator, 8) if denominator != 0 else 0.0
        else:
            raise Exception(f"Метод дефаззификации {self.def_method} не реализован")
