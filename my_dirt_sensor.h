#ifndef MY_DIRT_SENSOR_H
#define MY_DIRT_SENSOR_H

#include "dirt_sensor.h"

class MyDirtSensor : public DirtSensor {
public:
    int dirtLevel() const override;
};

#endif // MY_DIRT_SENSOR_H
