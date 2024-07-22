#ifndef MY_SIMULATOR_H
#define MY_SIMULATOR_H

#include "my_algorithm.h"
#include "my_wall_sensor.h"
#include "my_dirt_sensor.h"
#include "my_battery_meter.h"
#include "my_house.h"
#include <string>
#include <vector>

class MySimulator {
public:
    MySimulator();
    void readHouseFile(const std::string& houseFilePath);
    void setAlgorithm(MyAlgorithm& algo);
    void run();

private:
    MyAlgorithm* algo;
    House house;
};

#endif // MY_SIMULATOR_H
