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

// using namespace hello;

// MamdaniFuzzySystem mamdani_fuzzy_system = ;

// Test test = Test();

// extern "C"
// {
//     void My_Function(float a)
//     {
//         a += 200;
//         test.print(a);
//     }
// }

// int main()
// {
// }

extern "C"
{
    float *calculateFuzzy(float error_left_motor, float derivative_left_motor, float error_right_motor, float derivative_right_motor, char *output_name)
    {

        auto t1 = high_resolution_clock::now();

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

        // 3. Outputs
        FuzzyVariable output = FuzzyVariable("output", 0.0, 1.0);
        output.addTerm(Small);
        output.addTerm(MedSmall);
        output.addTerm(Med);
        output.addTerm(MedBig);
        output.addTerm(Big);

        // 4. Create system
        MamdaniFuzzySystem mamdani_fuzzy_system = MamdaniFuzzySystem({error, derivative_error}, {output});

        // 5. Add rules
        mamdani_fuzzy_system.addRule("if input1 is NB and input2 is DF then output is S");

        mamdani_fuzzy_system.addRule("if input1 is NB and input2 is DS then output is S");

        mamdani_fuzzy_system.addRule("if input1 is NB and input2 is M then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is NB and input2 is InS then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is NB and input2 is InF then output is M");

        mamdani_fuzzy_system.addRule("if input1 is NS and input2 is DF then output is S");

        mamdani_fuzzy_system.addRule("if input1 is NS and input2 is DS then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is NS and input2 is M then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is NS and input2 is InS then output is M");

        mamdani_fuzzy_system.addRule("if input1 is NS and input2 is InF then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is Z and input2 is DF then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is Z and input2 is DS then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is Z and input2 is M then output is M");

        mamdani_fuzzy_system.addRule("if input1 is Z and input2 is InS then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is Z and input2 is InF then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is PS and input2 is DF then output is MS");

        mamdani_fuzzy_system.addRule("if input1 is PS and input2 is DS then output is M");

        mamdani_fuzzy_system.addRule("if input1 is PS and input2 is M then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is PS and input2 is InS then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is PS and input2 is InF then output is B");

        mamdani_fuzzy_system.addRule("if input1 is PB and input2 is DF then output is M");

        mamdani_fuzzy_system.addRule("if input1 is PB and input2 is DS then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is PB and input2 is M then output is MB");

        mamdani_fuzzy_system.addRule("if input1 is PB and input2 is InS then output is B");

        mamdani_fuzzy_system.addRule("if input1 is PB and input2 is InF then output is B");
        // mamdani_fuzzy_system.printAllRules();

        // 6. Left motor
        mamdani_fuzzy_system.addInputValue("input1", error_left_motor);
        mamdani_fuzzy_system.addInputValue("input2", derivative_left_motor);

        mamdani_fuzzy_system.calculate(33);

        std::map<std::string, float> result_left_motor = mamdani_fuzzy_system.getResultMap();

        mamdani_fuzzy_system.addInputValue("input1", error_right_motor);
        mamdani_fuzzy_system.addInputValue("input2", derivative_right_motor);

        mamdani_fuzzy_system.calculate(33);

        std::map<std::string, float> result_right_motor = mamdani_fuzzy_system.getResultMap();

        // 7. Right motor

        // // 8. Print all results

        // std::cout << std::endl;

        // for (auto input : mamdani_fuzzy_system.getInputVariables())
        // {
        //     std::cout << input.getName() << ": " << input.getInputValue() << std::endl;
        // }

        // std::cout << std::endl;

        // for (auto pair : map_of_result)
        // {
        //     std::cout << "Output \"" << pair.first << "\": " << pair.second << std::endl;
        // }

        // std::cout << std::endl;

        // auto ms_int = duration_cast<milliseconds>(t2 - t1);

        auto t2 = high_resolution_clock::now();
        // /* Getting number of milliseconds as a double. */
        duration<double, std::milli> ms_double = t2 - t1;

        // std::cout << "Runtime: " << ms_double.count() << "ms\n";
        // std::cout << map_of_result.find("output")->second << std::endl;

        // std::cout << output_name << std::endl;

        float *list_of_result = (float *)malloc(sizeof(float) * 2);
        list_of_result[0] = result_left_motor.find(output_name)->second;
        list_of_result[1] = result_right_motor.find(output_name)->second;
        return list_of_result;
    }
}
