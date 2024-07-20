#include "my_algorithm.h"

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
    // Implement the logic to determine the next step based on sensor data
    // Placeholder implementation
    if (wallsSensor->isWall(Direction::North)) {
        return Step::Stay;
    } else {
        return Step::North;
    }
}
