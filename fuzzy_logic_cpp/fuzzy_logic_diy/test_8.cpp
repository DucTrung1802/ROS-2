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
    // Term for error
    Term NegBig = Term("NB", new TriangularMF(-200, -140, -70));
    Term NegSmall = Term("NS", new TriangularMF(-105, -40, 0));
    Term Zero = Term("Z", new TriangularMF(-35, 0, 35));
    Term PosSmall = Term("PS", new TriangularMF(0, 40, 105));
    Term PosBig = Term("PB", new TriangularMF(70, 140, 200));
    // Term for derivative error
    Term DecFast = Term("DF", new TriangularMF(-42000, -28000, -14000));
    Term DecSlow = Term("DS", new TriangularMF(-21000, -7000, 0));
    Term Maintain = Term("M", new TriangularMF(-5000, 0, 5000));
    Term Incslow = Term("InS", new TriangularMF(0, 7000, 21000));
    Term IncFast = Term("InF", new TriangularMF(14000, 28000, 42000));
    // Term for output
    Term Small = Term("S", new TriangularMF(-0.25, 0, 25));
    Term MedSmall = Term("MS", new TriangularMF(0, 0.25, 0.5));
    Term Med = Term("M", new TriangularMF(0.25, 0.5, 0.75));
    Term MedBig = Term("MB", new TriangularMF(0.5, 0.75, 1));
    Term Big = Term("B", new TriangularMF(0.75, 1, 1.25));
    // 2. Inputs
    FuzzyVariable error = FuzzyVariable("input1", -140, 140);
    error.addTerm(NegBig);
    error.addTerm(NegSmall);
    error.addTerm(Zero);
    error.addTerm(PosSmall);
    error.addTerm(PosBig);

    FuzzyVariable derivative_error = FuzzyVariable("input2", -28000, 28000);
    derivative_error.addTerm(DecFast);
    derivative_error.addTerm(DecSlow);
    derivative_error.addTerm(Maintain);
    derivative_error.addTerm(Incslow);
    derivative_error.addTerm(IncFast);
    // FuzzyVariable input_3 = FuzzyVariable("input2", 0.0, 1.0);
    // input_3.addTerm(term_1);
    // input_3.addTerm(term_2);
    // input_3.addTerm(term_3);

    // 3. Outputs
    FuzzyVariable output_ = FuzzyVariable("output", 0.0, 1.0);
    output.addTerm(Small);
    output.addTerm(MedSmall);
    output.addTerm(Med);
    output.addTerm(MedBig);
    output.addTerm(Big);
    // 4. Create system
    MamdaniFuzzySystem mamdani_fuzzy_system = MamdaniFuzzySystem({error, derivative_error}, {output});

    // 5. Add rules
    mamdani_fuzzy_system.addRule("if input1 is mf1 and input2 is mf1 then output is mf1");

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