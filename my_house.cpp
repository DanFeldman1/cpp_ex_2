#include "my_house.h"
#include <iostream>
#include <fstream>
#include <algorithm>


bool House::parseHouseFile(const std::string& houseFilePath) {
    std::ifstream file(houseFilePath);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << houseFilePath << std::endl;
        return false;
    }

    std::string line;
    int rowNum = 0;

    // Helper function to trim spaces around '='
    auto parseParameter = [](const std::string& line) {
        size_t pos = line.find('=');
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        key.erase(std::remove_if(key.begin(), key.end(), ::isspace), key.end());
        value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
        return std::make_pair(key, value);
    };

    // Read the first line
    std::getline(file, line);
    rowNum++;

    // Read and parse the parameters
    while (std::getline(file, line)) {
        if (line.find('=') != std::string::npos) {
            auto param = parseParameter(line);

            switch (rowNum) {
                case 1:
                    if (param.first == "MaxSteps") { this->maxSteps = std::stoi(param.second); }
                    else { file.close(); return false; }
                    break;
                    
                case 2:
                    if (param.first == "MaxBattery") { this->maxBattery = std::stoi(param.second); }
                    else { file.close(); return false; }
                    break;

                case 3:
                    if (param.first == "Rows") { 
                        this->rows = std::stoi(param.second); 
                        houseMap.resize(this->rows);
                    } else { 
                        file.close(); 
                        return false; 
                    }
                    break;

                case 4:
                    if (param.first == "Cols") { 
                        this->cols = std::stoi(param.second); 
                        rowNum = -1;
                    } else { 
                        file.close(); 
                        return false; 
                    }
                    break;

                default:
                    file.close(); 
                    return false;
            }
        } else if (!line.empty()) {
            // Reading the house layout
            if (rowNum < this->rows) {
                houseMap[rowNum].resize(this->cols, ' ');

                for (int j = 0; j < this->cols && j < int(line.size()); ++j) {
                    houseMap[rowNum][j] = line[j];
                }
            }
        }

        rowNum++;
    }

    file.close();
    
    // Fill empty rows with spaces
    for (int i = rowNum; i < this->rows; i++) {
        houseMap[i].resize(this->cols, ' ');
    }

    if (!isValidHouse()) {
        std::cerr << "Invalid number of docking stations in the house" << std::endl;
        return false;
    }

    setStartingPosition();
    printHouse();
    return true;
}

bool House::isValidHouse() {
    int dockingStationCnt = 0;

    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            if (houseMap[i][j] == 'D') {
                dockingStationCnt++;
            }
        }
    }

    return dockingStationCnt == 1;
}

void House::setStartingPosition() {
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            if (houseMap[i][j] == 'D') {
                this->rowPosition = i;
                this->colPosition = j;
                this->dockingRowPosition = i;
                this->dockingColPosition = j;
                return;
            }
        }
    }
}

void House::printHouse() const {
    std::cout << "House parameters:" << std::endl;
    std::cout << "MaxSteps: " << maxSteps << std::endl;
    std::cout << "MaxBattery: " << maxBattery << std::endl;
    std::cout << "Rows: " << rows << std::endl;
    std::cout << "Cols: " << cols << std::endl;
    std::cout << "Layout:" << std::endl;
    
    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            std::cout << houseMap[i][j];
        }
        std::cout << std::endl;
    }
}

int House::getTotalDirt() const {
    int totalDirt = 0;

    for (int i = 0; i < this->rows; i++) {
        for (int j = 0; j < this->cols; j++) {
            if (houseMap[i][j] >= '0' && houseMap[i][j] <= '9') {
                totalDirt += houseMap[i][j] - '0';
            }
        }
    }

    return totalDirt;
}