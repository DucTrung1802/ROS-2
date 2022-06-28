#include "FuzzyRule.h"

using namespace FLD;

void FuzzyRule::addAntecedentList(std::list<std::list<std::string>> antecedent_list)
{
    this->antecedent_list = antecedent_list;
}
std::list<std::list<std::string>> FuzzyRule::getAntecedentList()
{
    return this->antecedent_list;
}