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

        void addAntecedentList(std::list<std::list<std::string>> antecedent_list);
        void addConsequentList(std::list<std::list<std::string>> consequent_list);

        std::list<std::list<std::string>> getAntecedentList();
        std::list<std::list<std::string>> getConsequentList();
    };
}

#endif