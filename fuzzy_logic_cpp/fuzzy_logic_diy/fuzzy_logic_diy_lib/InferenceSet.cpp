#include "InferenceSet.h"

using namespace FLD;

InferenceSet::InferenceSet(std::string output_var_name, std::map<std::string, float> mf_set)
{
    this->output_var_name = output_var_name;
    this->mf_set = mf_set;
}