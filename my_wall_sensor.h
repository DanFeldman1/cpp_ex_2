#ifndef MY_WALL_SENSOR_H
#define MY_WALL_SENSOR_H

#include "wall_sensor.h"
#include "my_house.h"

class MyWallSensor : public WallsSensor {
public:
    MyWallSensor(House house);
    bool isWall(Direction d) const override;

private:
    House house;
};

#endif // MY_WALL_SENSOR_H
