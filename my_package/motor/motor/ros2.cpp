#include <chrono>
#include <thread>
#include "fuzzy_logic/Term.h"
#include "fuzzy_logic/FuzzyVariable.h"
#include "fuzzy_logic/RuleHandler.h"
#include "fuzzy_logic/MamdaniFuzzySystem.h"

using namespace FLD;
using std::chrono::duration;
using std::chrono::high_resolution_clock;

void evaluateValue(MamdaniFuzzySystem system, float error, float derivative, float &result, std::string output_name)
{
    system.addInputValue("input1", error);
    system.addInputValue("input2", derivative);

    system.calculate(25);

    result = system.getResultMap().find(output_name)->second;
}

extern "C"
{
    float *calculateFuzzy(float error_left_motor, float derivative_left_motor, float error_right_motor, float derivative_right_motor, char *output_name)
    {

        // auto t1 = high_resolution_clock::now();

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
        Term Small = Term("S", new TriangularMF(0.325, 0.5, 0.675));
        Term MedSmall = Term("MS", new TriangularMF(0.5, 0.675, 0.85));
        Term Med = Term("M", new TriangularMF(0.675, 0.85, 1.025));
        Term MedBig = Term("MB", new TriangularMF(0.85, 1.025, 1.2));
        Term Big = Term("B", new TriangularMF(1.025, 1.2, 1.375));

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
        FuzzyVariable output = FuzzyVariable("output", 0.5, 1.2);
        output.addTerm(Small);
        output.addTerm(MedSmall);
        output.addTerm(Med);
        output.addTerm(MedBig);
        output.addTerm(Big);

        // auto t2 = high_resolution_clock::now();

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

        // auto t3 = high_resolution_clock::now();

        float left_motor_result = 0.0;
        float right_motor_result = 0.0;

        // auto t1 = high_resolution_clock::now();

        std::thread th1(evaluateValue, mamdani_fuzzy_system, error_left_motor, derivative_left_motor, std::ref(left_motor_result), output_name);

        std::thread th2(evaluateValue, mamdani_fuzzy_system, error_right_motor, derivative_right_motor, std::ref(right_motor_result), output_name);

        th1.join();
        th2.join();

        // auto t2 = high_resolution_clock::now();

        // mamdani_fuzzy_system.addInputValue("input1", error_motor);
        // mamdani_fuzzy_system.addInputValue("input2", derivative_motor);

        // mamdani_fuzzy_system.calculate(40);

        // auto t4 = high_resolution_clock::now();

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

        // auto t2 = high_resolution_clock::now();
        // /* Getting number of milliseconds as a double. */
        // duration<double, std::milli> term = t2 - t1;
        // duration<double, std::milli> rule = t3 - t2;
        // duration<double, std::milli> calculate = t4 - t3;
        // duration<double, std::milli> total = t4 - t1;

        // std::cout << "Runtime: " << term.count() << "ms\n";
        // std::cout << "Rule: " << rule.count() << "ms\n";
        // std::cout << "Calculate: " << calculate.count() << "ms\n";
        // std::cout << "Total: " << total.count() << "ms\n";
        // std::cout << map_of_result.find("output")->second << std::endl;

        // std::cout << output_name << std::endl;

        float *list_of_result = (float *)malloc(sizeof(float) * 2);
        list_of_result[0] = left_motor_result;
        list_of_result[1] = right_motor_result;

        return list_of_result;
    }
}
