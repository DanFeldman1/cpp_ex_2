#ifndef MY_HOUSE_H
#define MY_HOUSE_H

#include <vector>
#include <string>

class House {
public:
    // Constructor
    House() : maxSteps(0), maxBattery(0), rows(0), cols(0) {}

    // Method to parse the input file and initialize the house parameters and layout
    bool parseHouseFile(const std::string& houseFilePath);

    bool isValidHouse();

    void printHouse() const;

    // Accessors
    int getMaxSteps() const { return maxSteps; }
    int getMaxBattery() const { return maxBattery; }
    int getRows() const { return rows; }
    int getCols() const { return cols; }
    const std::vector<std::vector<char>>& getHouseMap() const { return houseMap; }

private:
    int maxSteps;
    int maxBattery;
    int rows;
    int cols;
    std::vector<std::vector<char>> houseMap; // 2D vector for house layout
};

#endif // MY_HOUSE_H
