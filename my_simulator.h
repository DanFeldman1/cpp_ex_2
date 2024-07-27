#ifndef MY_SIMULATOR_H
#define MY_SIMULATOR_H

#include "my_algorithm.h"
#include "my_wall_sensor.h"
#include "my_dirt_sensor.h"
#include "my_battery_meter.h"
#include "my_house.h"
#include <string>
#include <vector>
#include <memory>

class MySimulator {
public:
    ~MySimulator();
    MySimulator();
    void readHouseFile(const std::string& houseFilePath);
    void setAlgorithm(MyAlgorithm& algo);
    void run();
    void executeStep(Step step);
    std::size_t getMaxSteps() const;

private:
    MyAlgorithm* algo;
    House house;
    int numSteps;
    int dirtLeft;
    std::vector<Step> steps;
    std::string status;

    char stepToChar(Step step);
    void writeOutputFile();
    bool isOnDocking();
};

#endif // MY_SIMULATOR_H
