#include "my_simulator.h"
#include <iostream>
#include <fstream>
#include <sstream>

MySimulator::MySimulator() : algo(nullptr), numSteps(0), dirtLeft(0), status("") {}

std::size_t MySimulator::getMaxSteps() const {
    return house.getMaxSteps();  // Assuming this method exists in the House class
}

void MySimulator::readHouseFile(const std::string& houseFilePath) {
    house = House();
    if (!house.parseHouseFile(houseFilePath)) {
        std::cerr << "Error parsing house file: " << houseFilePath << std::endl;
        return;
    }
}

void MySimulator::setAlgorithm(MyAlgorithm& algo) {
    this->algo = &algo;
    this->algo->setMaxSteps(house.getMaxSteps());
    this->algo->setWallsSensor(MyWallSensor(house));
    this->algo->setDirtSensor(MyDirtSensor(house));
    this->algo->setBatteryMeter(MyBatteryMeter(house.getMaxBattery(), house.getMaxBattery()));
}

char MySimulator::stepToChar(Step step) {
    switch (step) {
        case Step::North: return 'N';
        case Step::East: return 'E';
        case Step::South: return 'S';
        case Step::West: return 'W';
        case Step::Stay: return 's';
        case Step::Finish: return 'F';
        default: return ' ';
    }
}

void MySimulator::executeStep(Step step) {
    switch (step) {
        case Step::North:
            this->house.setColPosition(this->house.getColPosition());
            this->house.setRowPosition(this->house.getRowPosition() - 1);
            break;
        case Step::East:            
            this->house.setColPosition(this->house.getColPosition() + 1);
            this->house.setRowPosition(this->house.getRowPosition());
            break;
        case Step::South:
            this->house.setColPosition(this->house.getColPosition());
            this->house.setRowPosition(this->house.getRowPosition() + 1);
            break;
        case Step::West:
            this->house.setColPosition(this->house.getColPosition() - 1);
            this->house.setRowPosition(this->house.getRowPosition());
            break;
        case Step::Stay:
            this->house.decDirtLevel();
            break;    
        case Step::Finish:
            std::cout << "Simulation finished!" << std::endl;
            break;
    }
}

void MySimulator::run() {
    if (!algo) {
        std::cerr << "Algorithm not set!" << std::endl;
        return;
    }

    while (numSteps < house.getMaxSteps()) {
        Step nextStep = algo->nextStep();
        //std::cout << numSteps << " Next step: " << stepToChar(nextStep) << std::endl;
        steps.push_back(nextStep);
        numSteps++;

        if (nextStep == Step::Finish) {
            status = "Finished";
            break;
        }

        executeStep(nextStep);
    }
    
    if (numSteps == house.getMaxSteps() && status != "Finished") {
        status = "Stopped";
    }

    dirtLeft = house.getTotalDirt();

    writeOutputFile();
}

void MySimulator::writeOutputFile() {
    std::ofstream outFile("output.txt");
    if (!outFile) {
        std::cerr << "Could not open output file" << std::endl;
        return;
    }

    outFile << "NumSteps = " << numSteps << std::endl;
    outFile << "DirtLeft = " << dirtLeft << std::endl;
    outFile << "Status = " << status << std::endl;
    outFile << "Steps:" << std::endl;

    for (Step step : steps) {
        outFile << stepToChar(step);
    }

    outFile << std::endl;
    outFile.close();
}
