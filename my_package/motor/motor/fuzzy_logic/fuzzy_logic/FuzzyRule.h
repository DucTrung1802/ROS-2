#ifndef FUZZYRULE_H
#define FUZZYRULE_H

#include <string>
#include <list>

namespace FLD
{
    class FuzzyRule
    {
    public:
        std::list<std::list<std::string>> antecedent_list;
        std::list<std::list<std::string>> consequent_list;
        std::string text_form_of_fuzzy_rule;

        void
        addAntecedentList(std::list<std::list<std::string>> antecedent_list);
        void addConsequentList(std::list<std::list<std::string>> consequent_list);

        std::list<std::list<std::string>> getAntecedentList();
        std::list<std::list<std::string>> getConsequentList();
        void setTextFormOfRule(std::string rule);
        std::string getTextFormOfRule();
    };
}

#endif