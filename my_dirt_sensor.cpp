#include "my_dirt_sensor.h"
#include <vector>
#include <iostream>

MyDirtSensor::MyDirtSensor(House house) : house(house) {}

int MyDirtSensor::dirtLevel() const {
    std::vector<std::vector<char>> houseMap = house.getHouseMap();
    int rowPos = house.getRowPosition();
    int colPos = house.getColPosition();
    char c = houseMap[rowPos][colPos];
    
    if (c >= '0' && c <= '9') {
        return c - '0';
    } 
    else if (c == 'D' || c == ' ') {
        return 0;
    }

    return -1;
}
