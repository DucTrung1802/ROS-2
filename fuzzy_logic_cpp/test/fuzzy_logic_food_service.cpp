// File: ObstacleAvoidance.cpp
#include <chrono>
#include "fl/Headers.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    using namespace fl;
    // Code automatically generated with fuzzylite 6.0.

    using namespace fl;

    Engine *engine = new Engine;
    engine->setName("service_Service");
    engine->setDescription("");

    InputVariable *food = new InputVariable;
    food->setName("food");
    food->setDescription("");
    food->setEnabled(true);
    food->setRange(0.000, 10.000);
    food->setLockValueInRange(false);
    food->addTerm(new Trapezoid("rancid", 0.000, 0.000, 2.000, 4.000));
    food->addTerm(new Trapezoid("find", 2.000, 4.000, 6.000, 8.000));
    food->addTerm(new Trapezoid("delicious", 6.000, 8.000, 10.000, 10.000));
    engine->addInputVariable(food);

    InputVariable *service = new InputVariable;
    service->setName("service");
    service->setDescription("");
    service->setEnabled(true);
    service->setRange(0.000, 10.000);
    service->setLockValueInRange(false);
    service->addTerm(new Gaussian("poor", 0.000, 5.000));
    service->addTerm(new Gaussian("good", 5.000, 5.000));
    service->addTerm(new Gaussian("excellent", 10.000, 5.000));
    engine->addInputVariable(service);

    OutputVariable *tip = new OutputVariable;
    tip->setName("tip");
    tip->setDescription("");
    tip->setEnabled(true);
    tip->setRange(0.000, 1.000);
    tip->setLockValueInRange(false);
    tip->setAggregation(new Maximum);
    tip->setDefuzzifier(new Centroid(100));
    tip->setDefaultValue(fl::nan);
    tip->setLockPreviousValue(false);
    tip->addTerm(new Triangle("low", 0.000, 5.000, 15.000));
    tip->addTerm(new Triangle("average", 5.000, 15.000, 25.000));
    tip->addTerm(new Triangle("high", 15.000, 25.000, 30.000));
    engine->addOutputVariable(tip);

    RuleBlock *mamdani = new RuleBlock;
    mamdani->setName("mamdani");
    mamdani->setDescription("");
    mamdani->setEnabled(true);
    mamdani->setConjunction(fl::null);
    mamdani->setDisjunction(fl::null);
    mamdani->setImplication(new AlgebraicProduct);
    mamdani->setActivation(new General);
    mamdani->addRule(Rule::parse("if service is poor of food is rancid then tip is low", engine));
    mamdani->addRule(Rule::parse("if service is good then tip is average", engine));
    mamdani->addRule(Rule::parse("if service is excellent or food is delicious then tip is high", engine));
    engine->addRuleBlock(mamdani);

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    scalar food_quality = 7.000;
    food->setValue(food_quality);

    scalar service_quality = 8.000;
    service->setValue(service_quality);

    auto start = high_resolution_clock::now();
    engine->process();
    auto stop = high_resolution_clock::now();

    FL_LOG("food_quality.input = " << Op::str(food_quality) << " and "
                                   << "service_quality.input = " << Op::str(service_quality)
                                   << " => "
                                   << "tip.output = " << Op::str(tip->getValue()));

    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Time taken by function: " << duration.count() << " microseconds" << endl;

    return 0;
}