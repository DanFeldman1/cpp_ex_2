#ifndef MY_SIMULATOR_H
#define MY_SIMULATOR_H

#include "my_algorithm.h"
#include "my_wall_sensor.h"
#include "my_dirt_sensor.h"
#include "my_battery_meter.h"
#include <string>

class MySimulator {
public:
    void readHouseFile(const std::string& houseFilePath);
    void setAlgorithm(MyAlgorithm& algo);
    void run();

private:
    MyAlgorithm* algo;
    MyWallSensor wallsSensor;
    MyDirtSensor dirtSensor;
    MyBatteryMeter batteryMeter;
    int maxSteps;
    int maxBattery;
};

#endif // MY_SIMULATOR_H
