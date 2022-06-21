from typing import List, Dict

class MadamniFuzzySystem(object):
    def __init__(self, input=None, output=None) -> None:
        self.__input: List[FuzzyVariable] = input if input is not None else []
        self.__output: List[FuzzyVariable] = output if output is not None else []
        self.__rules: List[FuzzyRule] = []
        
    def addRules(self, fuzzy_rule: FuzzyRule) -> None:
        """Add rule to rule list

        Args:
            fuzzy_rule (FuzzyRule)
        """
        

    def parseRules(self, rule: str) -> FuzzyRule:
        """Parse rule from text

        Args:
            rule (str): rule in text view

        Returns:
            FuzzyRule: fuzzy rule
        """
        pass
    