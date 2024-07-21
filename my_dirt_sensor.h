#ifndef MY_DIRT_SENSOR_H
#define MY_DIRT_SENSOR_H

#include "dirt_sensor.h"

class MyDirtSensor : public DirtSensor {
public:
    MyDirtSensor();
    int dirtLevel() const override;

private:
    // Add any private members or helper functions here
};

#endif // MY_DIRT_SENSOR_H
