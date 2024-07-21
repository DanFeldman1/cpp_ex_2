#include "my_algorithm.h"

MyAlgorithm::MyAlgorithm()
    : maxSteps(0), wallsSensor(nullptr), dirtSensor(nullptr), batteryMeter(nullptr) {
}

void MyAlgorithm::setMaxSteps(std::size_t maxSteps) {
    this->maxSteps = maxSteps;
}

void MyAlgorithm::setWallsSensor(const WallsSensor& sensor) {
    this->wallsSensor = &sensor;
}

void MyAlgorithm::setDirtSensor(const DirtSensor& sensor) {
    this->dirtSensor = &sensor;
}

void MyAlgorithm::setBatteryMeter(const BatteryMeter& meter) {
    this->batteryMeter = &meter;
}

Step MyAlgorithm::nextStep() {
    // Implement your algorithm here
    return Step::Finish;
}
