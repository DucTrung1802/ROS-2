#include <iostream>
#include <chrono>
#include "fuzzy_logic/Term.h"
#include "fuzzy_logic/FuzzyVariable.h"
#include "fuzzy_logic/RuleHandler.h"
#include "fuzzy_logic/MamdaniFuzzySystem.h"

using namespace FLD;
using namespace std;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

int main()
{

    // 1. List all terms
    Term term_1 = Term("mf1", new TriangularMF(0.0, 0.0, 0.5));
    Term term_2 = Term("mf2", new TriangularMF(0.0, 0.5, 1.0));
    Term term_3 = Term("mf3", new TriangularMF(0.5, 1.0, 1.0));

    // 2. Inputs
    FuzzyVariable input_1 = FuzzyVariable("input1", 0.0, 1.0);
    input_1.addTerm(term_1);
    input_1.addTerm(term_2);
    input_1.addTerm(term_3);

    FuzzyVariable input_2 = FuzzyVariable("input2", 0.0, 1.0);
    input_2.addTerm(term_1);
    input_2.addTerm(term_2);
    input_2.addTerm(term_3);

    // FuzzyVariable input_3 = FuzzyVariable("input2", 0.0, 1.0);
    // input_3.addTerm(term_1);
    // input_3.addTerm(term_2);
    // input_3.addTerm(term_3);

    // FuzzyVariable input_3 = FuzzyVariable("input3", 0.0, 1.0);
    // input_3.addTerm(term_1);
    // input_3.addTerm(term_2);
    // input_3.addTerm(term_3);

    // FuzzyVariable input_4 = FuzzyVariable("input4", 0.0, 1.0);
    // input_4.addTerm(term_1);
    // input_4.addTerm(term_2);
    // input_4.addTerm(term_3);

    // 3. Outputs
    FuzzyVariable output_1 = FuzzyVariable("output1", 0.0, 1.0);
    output_1.addTerm(term_1);
    output_1.addTerm(term_2);
    output_1.addTerm(term_3);

    // 4. Create system
    MamdaniFuzzySystem mamdani_fuzzy_system = MamdaniFuzzySystem({input_1, input_2}, {output_1});

    // 5. Add rules
    mamdani_fuzzy_system.addRule("if input1 is mf1 and input2 is mf1 then output1 is mf1");

    mamdani_fuzzy_system.addRule("if input1 is mf2 and input2 is mf2 then output1 is mf2");

    mamdani_fuzzy_system.addRule("if input1 is mf3 and input2 is mf3 then output1 is mf3");

    mamdani_fuzzy_system.addRule("if input1 is mf1 and input2 is mf2 then output1 is mf3");

    // mamdani_fuzzy_system.printAllRules();

    auto t1 = high_resolution_clock::now();

    // 6. Input values for input variables
    mamdani_fuzzy_system.addInputValue("input1", 0.142);
    mamdani_fuzzy_system.addInputValue("input2", 0.659);

    // 7. Calculate
    mamdani_fuzzy_system.calculate(100);

    auto t2 = high_resolution_clock::now();

    // 8. Print all results
    std::map<std::string, float> map_of_result = mamdani_fuzzy_system.getResultMap();

    std::cout << std::endl;

    for (auto input : mamdani_fuzzy_system.getInputVariables())
    {
        std::cout << input.getName() << ": " << input.getInputValue() << std::endl;
    }

    std::cout << std::endl;

    for (auto pair : map_of_result)
    {
        std::cout << "Output \"" << pair.first << "\": " << pair.second << std::endl;
    }

    std::cout << std::endl;

    auto ms_int = duration_cast<milliseconds>(t2 - t1);

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << "Runtime: " << ms_double.count() << "ms\n";

    return 0;
}