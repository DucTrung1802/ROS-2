"""
Luferov Victor <lyferov@yandex.ru>

Sugeno Fuzzy System
"""
import cython
from typing import Dict, List
from collections import defaultdict
from .generic_fs import GenericFuzzySystem
from .rules import FuzzyRule
from .rule_parser import RuleParser
from .variables import FuzzyVariable, SugenoVariable, SugenoFunction
from .terms import Term
from .type import AndMethod, OrMethod


class SugenoFuzzySystem(GenericFuzzySystem):
    def __init__(
        self,
        inp: List[FuzzyVariable] = List[FuzzyVariable],
        out: List[SugenoVariable] = List[SugenoVariable],
        am: AndMethod = AndMethod.PROD,
        om: OrMethod = OrMethod.MAX,
    ):
        """
        Конструктор создания нечеткой переменной сугено
        :param inp: входящие переменные
        :param out: выходные переменные
        :param am: метод И
        :param om: метод ИЛИ
        """
        self.out: List[SugenoVariable] = out
        super().__init__(inp, am, om)

    def output_by_name(self, name: cython.char) -> SugenoVariable:
        """
        Ищем выходную переменную по имени
        :param name: имя переменной
        :return: переменная "variable.SugenoVariable"
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

    def evaluate_functions(
        self, iv: Dict[FuzzyVariable, cython.double]
    ) -> Dict[SugenoVariable, Dict[SugenoFunction, cython.double]]:
        return {
            variable: {sf: sf.evaluate(iv) for sf in variable.functions}
            for variable in self.out
        }

    def combine_result(
        self,
        rw: Dict[FuzzyRule, cython.double],
        fr: Dict[SugenoVariable, Dict[SugenoFunction, cython.double]],
    ) -> Dict[SugenoVariable, cython.double]:
        """
        Объединяем результаты функцию и правил
        :param rw: ruleWeights - весовые правила, результаты вычислений
        :param fr: function result - результат вычисления функций
        :return: значения выходных функций
        """
        numerator: Dict[SugenoVariable, cython.double] = defaultdict(cython.double)
        denominator: Dict[SugenoVariable, cython.double] = defaultdict(cython.double)
        for out in self.out:
            numerator[out] = 0.0
            denominator[out] = 0.0

        for rule, weight in rw.items():
            variable: SugenoVariable = rule.conclusion.variable
            z: cython.double = fr[variable][rule.conclusion.term]
            numerator[variable] += z * weight
            denominator[variable] += weight

        return {
            out: 0.0 if denominator[out] == 0.0 else numerator[out] / denominator[out]
            for out in self.out
        }

    def calculate(
        self, input_values: Dict[FuzzyVariable, cython.double]
    ) -> Dict[SugenoVariable, cython.double]:
        if len(self.rules) == 0:
            raise Exception("Должно быть как минимум одно правило")
        fi: Dict[FuzzyVariable, Dict[Term, cython.double]] = self.fuzzify(
            input_values
        )  # Шаг фаззификации
        rw: Dict[FuzzyRule, cython.double] = self.evaluate_conditions(
            fi
        )  # Агрегация подусловий
        fr: Dict[
            SugenoVariable, Dict[SugenoFunction, cython.double]
        ] = self.evaluate_functions(input_values)
        result: Dict[SugenoVariable, cython.double] = self.combine_result(rw, fr)
        return result
