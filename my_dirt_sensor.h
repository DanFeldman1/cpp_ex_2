#ifndef MY_DIRT_SENSOR_H
#define MY_DIRT_SENSOR_H

#include "dirt_sensor.h"
#include "my_house.h"

class MyDirtSensor : public DirtSensor {
public:
    MyDirtSensor(House house);
    int dirtLevel() const override;

private:
    House house;
};

#endif // MY_DIRT_SENSOR_H
