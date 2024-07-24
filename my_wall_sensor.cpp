#include "my_wall_sensor.h"
#include <vector>

MyWallSensor::MyWallSensor(House house) : house(house) {}

bool MyWallSensor::isWall(Direction d) const {
    int rowPos = house.getRowPosition();
    int colPos = house.getColPosition();
    int rowsNum = house.getRows();
    int colsNum = house.getCols();
    std::vector<std::vector<char>> houseMap = house.getHouseMap();

    if (d == Direction::North) {
        if (rowPos == 0) {
            return true;
        }

        if (houseMap[rowPos - 1][colPos] == 'W') {
            return true;
        }

        return false;
    }
    
    else if (d == Direction::South) {
        if (rowPos == rowsNum - 1) {
            return true;
        }

        if (houseMap[rowPos + 1][colPos] == 'W') {
            return true;
        }

        return false;
    }
    
    else if (d == Direction::East) {
        if (colPos == colsNum - 1) {
            return true;
        }

        if (houseMap[rowPos][colPos + 1] == 'W') {
            return true;
        }

        return false;
    }
    
    else if (d == Direction::West) {
        if (colPos == 0) {
            return true;
        }

        if (houseMap[rowPos][colPos - 1] == 'W') {
            return true;
        }

        return false;
    }

    return false;
}
