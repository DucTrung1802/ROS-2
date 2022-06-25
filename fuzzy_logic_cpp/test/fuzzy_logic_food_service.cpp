// File: ObstacleAvoidance.cpp
#include <chrono>
#include "fl/Headers.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    using namespace fl;
    // Code automatically generated with fuzzylite 6.0.

    Engine *engine = new Engine;
    engine->setName("service_Service");
    engine->setDescription("");

    InputVariable *input1 = new InputVariable;
    input1->setName("input1");
    input1->setDescription("");
    input1->setEnabled(true);
    input1->setRange(0.000, 1.000);
    input1->addTerm(new Triangle("mf1", 0.0, 0.0, 0.5));
    input1->addTerm(new Triangle("mf2", 0.0, 0.5, 1.0));
    input1->addTerm(new Triangle("mf3", 0.5, 1.0, 1.0));
    engine->addInputVariable(input1);

    InputVariable *input2 = new InputVariable;
    input2->setName("input2");
    input2->setDescription("");
    input2->setEnabled(true);
    input2->setRange(0.000, 1.000);
    input2->addTerm(new Triangle("mf1", 0.0, 0.0, 0.5));
    input2->addTerm(new Triangle("mf2", 0.0, 0.5, 1.0));
    input2->addTerm(new Triangle("mf3", 0.5, 1.0, 1.0));
    engine->addInputVariable(input2);

    OutputVariable *output = new OutputVariable;
    output->setName("output");
    output->setDescription("");
    output->setEnabled(true);
    output->setRange(0.000, 1.000);
    output->setLockValueInRange(false);
    output->setAggregation(new Maximum);
    output->setDefuzzifier(new Centroid(100));
    output->setDefaultValue(fl::nan);
    output->setLockPreviousValue(false);
    output->addTerm(new Triangle("mf1", 0.0, 0.0, 0.5));
    output->addTerm(new Triangle("mf2", 0.0, 0.5, 1.0));
    output->addTerm(new Triangle("mf3", 0.5, 1.0, 1.0));
    engine->addOutputVariable(output);

    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(new Minimum);
    mamdani->setDisjunction(new Maximum);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if input1 is mf1 and input2 is mf1 then output is mf1", engine));
    mamdani->addRule(Rule::parse("if input1 is mf2 and input2 is mf2 then output is mf2", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    scalar input1_val = 0.333;
    input1->setValue(input1_val);

    scalar input2_val = 0.2;
    input2->setValue(input2_val);

    auto start = high_resolution_clock::now();
    engine->process();
    auto stop = high_resolution_clock::now();

    FL_LOG("input1 = " << Op::str(input1_val) << " and "
                       << "input2 = " << Op::str(input2_val)
                       << " => "
                       << "output = " << Op::str(output->getValue()));

    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Time taken by function: " << duration.count() << " microseconds" << endl;

    return 0;
}