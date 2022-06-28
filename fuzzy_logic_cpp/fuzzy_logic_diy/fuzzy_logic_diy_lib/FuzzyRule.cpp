#include "FuzzyRule.h"

using namespace FLD;

void FuzzyRule::addAntecedentList(std::list<std::list<std::string>> antecedent_list)
{
    this->antecedent_list = antecedent_list;
}