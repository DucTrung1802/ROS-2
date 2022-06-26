#include "MamdaniFuzzySystem.h"

using namespace FLD;

MamdaniFuzzySystem::MamdaniFuzzySystem(std::list<FuzzyVariable> input_variables, std::list<FuzzyVariable> output_variables)
{
    this->input_variables = input_variables;
    this->output_variables = output_variables;
}