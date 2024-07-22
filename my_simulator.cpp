#include "my_simulator.h"
#include <iostream>
#include <fstream>

MySimulator::MySimulator() {}

void MySimulator::readHouseFile(const std::string& houseFilePath) {
    house = House();
    if (!house.parseHouseFile(houseFilePath)) {
        std::cerr << "Error parsing house file: " << houseFilePath << std::endl;
        return;
    }

    house.printHouse();
}

void MySimulator::setAlgorithm(MyAlgorithm& algo) {
    this->algo = &algo;
    algo.setMaxSteps(house.getMaxSteps());
    algo.setWallsSensor(wallsSensor);
    algo.setDirtSensor(dirtSensor);
    algo.setBatteryMeter(batteryMeter);
}

void MySimulator::run() {
    // Run the simulation
}
