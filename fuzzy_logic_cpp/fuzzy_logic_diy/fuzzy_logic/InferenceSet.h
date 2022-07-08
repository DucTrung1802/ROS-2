#ifndef INFERENCESET_H
#define INFERENCESET_H

#include <iostream>
#include <map>
#include <string>

namespace FLD
{
    class InferenceSet
    {
    public:
        std::string output_var_name;
        std::map<std::string, float> mf_set;

        InferenceSet(std::string output_var_name, std::map<std::string, float> mf_set);
        std::string getOutputVarName();
        void setValueMf(std::string mf_name, float value);
        float getValueMf(std::string mf_name);
        void printInferenceSet();
    };
}

#endif