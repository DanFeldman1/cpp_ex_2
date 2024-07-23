#include "my_algorithm.h"
#include "enums.h"

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>

MyAlgorithm::MyAlgorithm() : 
    currentPosition({0, 0}),
    dockingStation({0, 0}),
    maxBattery(0),
    remainingSteps(0),
    initialized(false),
    cleaning(false),
    charging(false),
    chargingSteps(0),
    explorationIndex(0) {}
    
void MyAlgorithm::setMaxSteps(std::size_t maxSteps) {
    this->remainingSteps = maxSteps;
}

void MyAlgorithm::setWallsSensor(const WallsSensor& sensor) {
    // Since WallsSensor is abstract, we need to cast to the concrete type
    const MyWallSensor& mySensor = dynamic_cast<const MyWallSensor&>(sensor);
    this->wallsSensor = std::make_unique<MyWallSensor>(mySensor);
}

void MyAlgorithm::setDirtSensor(const DirtSensor& sensor) {
    // Since DirtSensor is abstract, we need to cast to the concrete type
    const MyDirtSensor& mySensor = dynamic_cast<const MyDirtSensor&>(sensor);
    this->dirtSensor = std::make_unique<MyDirtSensor>(mySensor);
}

void MyAlgorithm::setBatteryMeter(const BatteryMeter& meter) {
    // Since BatteryMeter is abstract, we need to cast to the concrete type
    const MyBatteryMeter& myMeter = dynamic_cast<const MyBatteryMeter&>(meter);
    this->batteryMeter = std::make_unique<MyBatteryMeter>(myMeter);
}

void MyAlgorithm::setMaxBattery(int maxBattery) {
    this->maxBattery = this->batteryMeter.get()->getMaxBattery();
}

void MyAlgorithm::initialize() {
    currentPosition = {0, 0};
    dockingStation = {0, 0};
    // batteryState = maxBattery; 
    cleaning = false;
    charging = false;
    directions = {Step::North, Step::South, Step::East, Step::West};
    initialized = false;
    /*walls.clear();
    notWalls.clear();

    for (Step step : directions) {
        if (isWall(step)) {
            walls.push_back(step);
            Position wallPos = calcNextCell(currentPosition, step);
            dynamicMap[wallPos] = 'W';
        } else {
            notWalls.push_back(step);
        }
    }*/
}

bool MyAlgorithm::isWall(Step step) {
    return this->wallsSensor.get()->isWall(convertStepToDirection(step));
} 

int MyAlgorithm::getBatteryState() {
    return this->batteryMeter.get()->getBatteryState();
}

int MyAlgorithm::getDirtLevel() {
    return this->dirtSensor.get()->dirtLevel();
}

void MyAlgorithm::chargeBattery() {
    this->batteryMeter.get()->chargeBattery();
}

void MyAlgorithm::decreaseBattery() {
    this->batteryMeter.get()->decreaseBattery();
}

Position MyAlgorithm::calcNextCell(Position current, Step dir) {
    switch (dir) {
        case Step::North:
            return {current.x, current.y + 1};
        case Step::South:
            return {current.x, current.y - 1};
        case Step::East:
            return {current.x + 1, current.y};
        case Step::West:
            return {current.x - 1, current.y};
        case Step::Stay:
            return current;
        default:
            return current;
    }
}

std::vector<Step> MyAlgorithm::findPathToDocking() {
    return bfsToDocking(currentPosition);
}

Step MyAlgorithm::nextStep() {
    try {
        if (!initialized) {
            initialize();
            initialized = true;
        }

        // Ensure we have enough steps to return to the docking station
        if (remainingSteps <= findPathToDocking().size()) {
            // If we're out of steps we finish the algorithm
            returningToFinish = true;
            cleaning = false;
        }

        // Handle returning to the docking station for finishing
        if (returningToFinish) {
            if (currentPosition == dockingStation) {
                returningToFinish = false;
                charging = false;
                return Step::Finish;
            }
            if (pathToDock.empty()) {
                pathToDock = bfsToDocking(currentPosition);
            }

            Step nextStep = pathToDock.front();
            pathToDock.erase(pathToDock.begin());
            currentPosition = calcNextCell(currentPosition, nextStep);
            decreaseBattery();
            remainingSteps--;
            return nextStep;
        }

        // Check if battery is low and initiate return to docking station if necessary
        if (getBatteryState() <= findPathToDocking().size()) {
            returningToCharge = true;
            cleaning = false;
        }

        // Handle returning to the docking station for recharging
        if (returningToCharge) {
            if (currentPosition == dockingStation) {
                returningToCharge = false;
                charging = true;
            } else {
                if (pathToDock.empty()) {
                    pathToDock = bfsToDocking(currentPosition);
                }

                Step nextStep = pathToDock.front();
                pathToDock.erase(pathToDock.begin());
                currentPosition = calcNextCell(currentPosition, nextStep);
                decreaseBattery();
                remainingSteps--;
                return nextStep;
            }
        }

        // Handle charging at the docking station
        if (charging) {
            if (getBatteryState() < maxBattery) {
                chargeBattery();
                remainingSteps--;
                return Step::Stay;
            } else {
                charging = false;
                walkingToNextCell = true;
            }
        }

        if (!walkingToNextCell) {
            walkingToNextCell = true;
            pathToNextCell = bfs(currentPosition, remainingSteps);
        }

        if (walkingToNextCell) {
            if (pathToNextCell.empty()) {
                walkingToNextCell = false;
                int dirtLevel = getDirtLevel();

                // Initializing area around cell
                dynamicMap[currentPosition] = dirtLevel;
                
                for (Step d : directions) {
                    Position newPos = calcNextCell(currentPosition, d);

                    if (isWall(d)) {
                        dynamicMap[newPos] = 'W';
                    }
                    else {
                        dynamicMap[newPos] = 'U';
                    }
                }

                if (dirtLevel > 0) {
                    cleaning = true;
                }
                else {
                    walkingToNextCell = true;
                    pathToNextCell = bfs(currentPosition, remainingSteps);
                }
            }
            // Peeling the path
            if (!cleaning) {
                Step nextStep = pathToNextCell.front();
                pathToNextCell.erase(pathToNextCell.begin());
                currentPosition = calcNextCell(currentPosition, nextStep);
                decreaseBattery();
                remainingSteps--;
                return nextStep;
            }
        }

        // Handle cleaning
        if (cleaning) {
            if (dynamicMap[currentPosition] > 0) {
                dynamicMap[currentPosition]--;
                decreaseBattery();
                remainingSteps--;
                if (dynamicMap[currentPosition] == 0) {
                    cleaning = false; // Done cleaning this tile
                    walkingToNextCell = true;
                }
                return Step::Stay; // Continue cleaning
            }
        }
    }

    catch (const std::exception& e) {
        std::cerr << "Error in nextStep: " << e.what() << std::endl;
        return Step::Stay; // Default to stay in case of an error
    }
}

Step MyAlgorithm::argmax(const std::unordered_map<Step, int, StepHash>& dict) {
    return std::max_element(dict.begin(), dict.end(),
                            [](const std::pair<Step, int>& a, const std::pair<Step, int>& b) {
                                return a.second < b.second;
                            })->first;
}

Direction convertStepToDirection(Step step) {
    switch (step) {
        case Step::North:
            return Direction::North;
        case Step::South:
            return Direction::South;
        case Step::East:
            return Direction::East;
        case Step::West:
            return Direction::West;
        default:
            std::cout << "Invalid step" << std::endl;
            return Direction::North;
    }
}

std::vector<Step> MyAlgorithm::bfs(const Position& start, int maxLength) {
    std::unordered_map<Position, Position, PositionHash> parent;  // To track the path
    std::unordered_map<Position, int, PositionHash> distance;  // To track the distance from the start
    std::queue<Position> q;  // Queue for BFS
    q.push(start);  // Start from the current position
    parent[start] = start;  // Start has no parent, mark it as its own parent
    distance[start] = 0;  // Start has distance 0

    while (!q.empty()) {
        Position current = q.front();
        q.pop();

        // Check if the current cell is unexplored or dirty
        char cellStatus = dynamicMap[current];

        if (cellStatus == 'U' || (cellStatus >= '1' && cellStatus <= '9')) {
            // Construct the path back to the start
            std::vector<Step> path;
            Position pathPos = current;
            while (pathPos != start) {
                Position prev = parent[pathPos];
                if (pathPos.x == prev.x + 1) path.push_back(Step::East);
                else if (pathPos.x == prev.x - 1) path.push_back(Step::West);
                else if (pathPos.y == prev.y + 1) path.push_back(Step::North);
                else if (pathPos.y == prev.y - 1) path.push_back(Step::South);
                pathPos = prev;
            }
            std::reverse(path.begin(), path.end());
            return path; // Return the path to the first unexplored or dirty cell
        }

        // Explore neighbors (North, South, East, West)
        for (Step step : directions) {
            Position next = calcNextCell(current, step);

            // Continue exploring only if the cell is not a wall and hasn't been visited yet
            if (dynamicMap.find(next) != dynamicMap.end() && dynamicMap[next] != 'W' && parent.find(next) == parent.end()) {
                int newDistance = distance[current] + 1;

                if (newDistance <= maxLength) {
                    parent[next] = current;
                    distance[next] = newDistance;
                    q.push(next);
                }
            }
        }
    }

    return {};  // Return an empty path if no unexplored or dirty cell is found
}

std::vector<Step> MyAlgorithm::bfsToDocking(const Position& start) {
    std::unordered_map<Position, Position, PositionHash> parent;
    std::queue<Position> q;
    q.push(start);
    parent[start] = start;

    while (!q.empty()) {
        Position current = q.front();
        q.pop();

        // Check if we've reached the docking station
        if (current == this->dockingStation) {
            std::vector<Step> path;
            Position pathPos = current;
            while (pathPos != start) {
                Position prev = parent[pathPos];
                if (pathPos.x == prev.x + 1) path.push_back(Step::East);
                else if (pathPos.x == prev.x - 1) path.push_back(Step::West);
                else if (pathPos.y == prev.y + 1) path.push_back(Step::South);
                else if (pathPos.y == prev.y - 1) path.push_back(Step::North);
                pathPos = prev;
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (Step step : directions) {
            Position next = calcNextCell(current, step);

            if (dynamicMap.find(next) != dynamicMap.end() && dynamicMap[next] != 'W' && parent.find(next) == parent.end()) {
                parent[next] = current;
                q.push(next);
            }
        }
    }

    return {}; // No path found
}