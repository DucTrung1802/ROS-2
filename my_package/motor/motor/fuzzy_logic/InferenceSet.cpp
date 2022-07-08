#include "InferenceSet.h"

using namespace FLD;

InferenceSet::InferenceSet(std::string output_var_name, std::map<std::string, float> mf_set)
{
    this->output_var_name = output_var_name;
    this->mf_set = mf_set;
}

std::string InferenceSet::getOutputVarName()
{
    return this->output_var_name;
}

void InferenceSet::setValueMf(std::string mf_name, float value)
{
    for (auto &mf : this->mf_set)
    {
        if (mf_name == mf.first)
        {
            mf.second = value;
            return;
        }
    }

    std::cout << "Unable to set value for inference set with output variable name is \"" << this->output_var_name << "\" since \"" << mf_name << "\" is not a name of any terms!" << std::endl;
}

float InferenceSet::getValueMf(std::string mf_name)
{
    for (auto mf : this->mf_set)
    {
        if (mf_name == mf.first)
        {
            return mf.second;
        }
    }

    std::cout << "Output variable \"" << this->output_var_name << "\" does not have any term name \"" << mf_name << "\"!" << std::endl;
    return 0.0;
}

void InferenceSet::printInferenceSet()
{
    std::cout << "Output variable name: "
              << "\"" << this->output_var_name << "\"" << std::endl;
    std::cout << "MF set:";
    for (auto mf : this->mf_set)
    {
        std::cout << " {"
                  << "\"" << mf.first << "\""
                  << ", " << mf.second << "} ";
    }
    std::cout << std::endl;
}
