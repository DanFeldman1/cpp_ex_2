#ifndef MY_WALL_SENSOR_H
#define MY_WALL_SENSOR_H

#include "wall_sensor.h"

class MyWallSensor : public WallsSensor {
public:
    MyWallSensor();
    bool isWall(Direction d) const override;

private:
    // Add any private members or helper functions here
};

#endif // MY_WALL_SENSOR_H
