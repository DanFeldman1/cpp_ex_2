#include "my_simulator.h"
#include <iostream>
#include <fstream>

MySimulator::MySimulator() : algo(nullptr), maxSteps(100) { // Example initialization
    // Initialization code
}

void MySimulator::readHouseFile(const std::string& houseFilePath) {
    // Read the house file and initialize sensors and other components
    std::ifstream houseFile(houseFilePath);
    if (!houseFile) {
        std::cerr << "Error opening house file: " << houseFilePath << std::endl;
        return;
    }
    // Process house file contents
    // ...
}

void MySimulator::setAlgorithm(MyAlgorithm& algo) {
    this->algo = &algo;
    algo.setMaxSteps(maxSteps);
    algo.setWallsSensor(wallsSensor);
    algo.setDirtSensor(dirtSensor);
    algo.setBatteryMeter(batteryMeter);
}

void MySimulator::run() {
    // Run the simulation
}
