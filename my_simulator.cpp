#include "my_simulator.h"
#include <iostream>
#include <fstream>
#include <sstream>


MySimulator::MySimulator() : algo(nullptr), numSteps(0), dirtLeft(0), status("WORKING") {}

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

void MySimulator::executeStep(Step step) {
    // The execution of the step
    switch (step) {
        case Step::North:
            // Move the robot North
            this->house.setColPosition(this->house.getColPosition());
            this->house.setRowPosition(this->house.getRowPosition() - 1);
            break;
        case Step::East:            
            // Move the robot South
            this->house.setColPosition(this->house.getColPosition() + 1);
            this->house.setRowPosition(this->house.getRowPosition());
            break;
        case Step::South:
            // Move the robot South
            this->house.setColPosition(this->house.getColPosition());
            this->house.setRowPosition(this->house.getRowPosition() + 1);
            break;
        case Step::West:
            // Move the robot South
            this->house.setColPosition(this->house.getColPosition() - 1);
            this->house.setRowPosition(this->house.getRowPosition());
            break;
        case Step::Stay:
            // Clean the current cell
            this->house.decDirtLevel();
            break;    
        case Step::Finish:
            // Finish the simulation
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
        Step next_step = algo->nextStep();
        steps.push_back(next_step);
        numSteps++;

        if(next_step == Step::Finish) {
            status = "Finished";
            break;
        }

        executeStep(next_step);
    }

    if (numSteps == house.getMaxSteps() && status != "Finished") {
        status = "Stopped";
    }

    dirtLeft = house.getTotalDirt();

    /* 
    for (int i = 0; i < house.getMaxSteps(); ++i) {
        Step next_step = algo->nextStep();
        executeStep(next_step);  // Implement the execute method to handle the step
    } */
}

void MySimulator::writeOutputFile(const std::string& outputFilePath) {
    std::ofstream outFile(outputFilePath);
    if (!outFile) {
        std::cerr << "Could not open output file!" << std::endl;
        return;
    }

    outFile << "NumSteps = " << numSteps << "\n";
    outFile << "DirtLeft = " << dirtLeft << "\n";
    outFile << "Status = " << status << "\n";
    outFile << "Steps:\n";

    for (Step step : steps) {
        switch (step) {
            case Step::North: outFile << 'N'; break;
            case Step::East: outFile << 'E'; break;
            case Step::South: outFile << 'S'; break;
            case Step::West: outFile << 'W'; break;
            case Step::Stay: outFile << 's'; break;
            case Step::Finish: outFile << 'F'; break;
        }
    }

    outFile.close();
}

std::size_t MySimulator::getMaxSteps() const {
    return house.getMaxSteps();  // Assuming this method exists in the House class
}