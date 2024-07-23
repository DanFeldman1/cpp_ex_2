#include "my_algorithm.h"
#include "enums.h"

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <queue>
#include <iostream>

MyAlgorithm::MyAlgorithm(): 
    currentPosition({0, 0}),
    dockingStation({0, 0}),
    initialized(false),
    cleaning(false),
    charging(false)
{}

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

// Initialize the algorithm, retruns whether the initalization was successful
bool MyAlgorithm::initialize() {
    // to check if we have already initialized
    initialized = false;
    // Step directions for advancing
    directions = {Step::North, Step::South, Step::East, Step::West};
    
    // Initalize the positions
    currentPosition = {0, 0};
    dockingStation = {0, 0};

    // The booleans we use to manage the state we're in
    cleaning = false;
    charging = false;
    walkToDockWhenFinished = false;
    walkToDockWhenLowBattery = false;
    isThereAPathToNextCell = false;

    walls.clear();
    notWalls.clear();

    for (Step step : directions) {
        Position wallPos = calcNextCell(currentPosition, step);
        if (isWall(step)) {
            walls.push_back(step);
            dynamicMap[wallPos] = 'W';
        } else {
            notWalls.push_back(step);
            dynamicMap[wallPos] = 'U';
        }
    }

    if (notWalls.empty()) {
        return false;
    }
    return true;
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

void MyAlgorithm::setAllStatesFalse() {
    cleaning = false;
    charging = false;
    walkToDockWhenFinished = false;
    walkToDockWhenLowBattery = false;
    isThereAPathToNextCell = false;
}

Step MyAlgorithm::nextStep() {
    try {
        if (!initialized) {
            bool nullCase = initialize();
            if (!nullCase) {
                throw std::runtime_error("No paths available from docking station");
            }
            initialized = true;
        }

        // Ensure we have enough steps to return to the docking station
        if (remainingSteps <= findPathToDocking().size()) {
            // If we're out of steps we finish the algorithm
            setAllStatesFalse();
            walkToDockWhenFinished = true;
        }

        // Handle returning to the docking station for finishing
        if (walkToDockWhenFinished) {
            if (currentPosition == dockingStation) {
                setAllStatesFalse();
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
            setAllStatesFalse();
            walkToDockWhenLowBattery = true;
        }

        // Handle returning to the docking station for recharging
        if (walkToDockWhenLowBattery) {
            if (currentPosition == dockingStation) {
                setAllStatesFalse();
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
                setAllStatesFalse();
                isThereAPathToNextCell = true;
            }
        }

        // Check if we already have a path to another cell, if not generate one
        if (!isThereAPathToNextCell) {
            isThereAPathToNextCell = true;
            // Using BFS generate a path to another cell
            pathToNextCell = bfs(currentPosition, remainingSteps);
        }

        // If we have a path to another cell, walk to it
        if (isThereAPathToNextCell) {
            // If we finished to walk to the next cell, 
            // either clean it if necessary, or proceed to the next cell.
            if (pathToNextCell.empty()) {
                isThereAPathToNextCell = false;
                int dirtLevel = getDirtLevel();

                // Initializing the area around the cell
                dynamicMap[currentPosition] = dirtLevel;
                
                for (Step d : directions) {
                    Position newPos = calcNextCell(currentPosition, d);
                    if (isWall(d)) {
                        dynamicMap[newPos] = 'W';
                    } else {
                        dynamicMap[newPos] = 'U';
                    }
                }

                // Check if the cell needs cleaning
                if (dirtLevel > 0) {
                    cleaning = true;
                    // If the cell is clean already, go to the next one.
                } else {
                    isThereAPathToNextCell = true;
                    // Using BFS generate a path to another cell
                    pathToNextCell = bfs(currentPosition, remainingSteps);
                }
            }
            // Peeling the path
            if (!cleaning) {
                // Pop the next step from the path
                Step nextStep = pathToNextCell.front();
                // Erase the step from the path
                pathToNextCell.erase(pathToNextCell.begin());
                // Update the position
                currentPosition = calcNextCell(currentPosition, nextStep);
                // Adjust the battery
                decreaseBattery();
                // Decrease the remaining steps
                remainingSteps--;

                return nextStep;
            }
        }

        // Handle cleaning
        if (cleaning) {
            // If the cell is dirty, clean it
            if (dynamicMap[currentPosition] > '0' && dynamicMap[currentPosition] <= '9') {
                // Decrease the dirt level
                dynamicMap[currentPosition]--;
                // Adjust the battery
                decreaseBattery();
                // Decrease the remaining steps
                remainingSteps--;

                // If after this cleaning, the cell is clean, go to the next cell
                if (dynamicMap[currentPosition] == 0) {
                    cleaning = false; // Done cleaning this tile
                    isThereAPathToNextCell = true; // Need to find a new path
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