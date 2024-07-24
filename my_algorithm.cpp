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
    maxBattery(0),
    remainingSteps(0),
    initialized(false),
    cleaning(false),
    charging(false),
    walkToDockWhenLowBattery(false),
    walkToDockWhenFinished(false)
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
    directions = {Step::North, Step::South, Step::East, Step::West};

    walls.clear();
    notWalls.clear();

    dynamicMap[dockingStation] = 'D';

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

    return !notWalls.empty();
}

bool MyAlgorithm::isWall(Step step) {
    return this->wallsSensor->isWall(convertStepToDirection(step));
} 

int MyAlgorithm::getBatteryState() {
    return this->batteryMeter->getBatteryState();
}

int MyAlgorithm::getDirtLevel() {
    return this->dirtSensor->dirtLevel();
}

void MyAlgorithm::chargeBattery() {
    this->batteryMeter->chargeBattery();
}

void MyAlgorithm::decreaseBattery() {
    this->batteryMeter->decreaseBattery();
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

Step MyAlgorithm::handleDockingFinish() {
    if (currentPosition == dockingStation) {
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

Step MyAlgorithm::handleDockingRecharge() {
    if (currentPosition == dockingStation) {
        return handleCharging();
    } 
    else {
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

Step MyAlgorithm::handleCharging() {
    if (getBatteryState() < maxBattery) {
        chargeBattery();
        remainingSteps--;
        return Step::Stay;
    } else { 
        // In case the battery is fully charged
        return handleWalkingToNextCell();
    }
}

Step MyAlgorithm::handleWalkingToNextCell() { 
    bool arrived = false;

    if (pathToNextCell.empty()) {
        arrived = true;
        int dirtLevel = getDirtLevel();

        // Initializing the cell and the area around the cell
        if (currentPosition != dockingStation) {
            dynamicMap[currentPosition] = '0' + dirtLevel;
        }

        for (Step d : directions) {
            Position newPos = calcNextCell(currentPosition, d);

            if (isWall(d)) {
                dynamicMap[newPos] = 'W';
            } else {
                if (dynamicMap.find(newPos) == dynamicMap.end()) 
                    dynamicMap[newPos] = 'U';
            }
        }

        // Check if the cell needs cleaning
        if (dirtLevel > 0) {
            cleaning = true;
            return handleCleaning();
        } else {
            // Using BFS generate a path to another cell under the limitation of the remaining steps
            pathToNextCell = generatePath();

            if (pathToNextCell.empty()) {
                // Since no path is found, either there is not enough battery or not enough steps
                            
                // If we're out of steps we finish the algorithm
                if (remainingSteps <= int(findPathToDocking().size())) 
                    return handleDockingFinish();

                // If we're out of battery we recharge
                if (getBatteryState() <= int(findPathToDocking().size())) 
                    return handleDockingRecharge();
            
            }
            arrived = false;
        }
    }

    if (!arrived) {
        Step nextStep = pathToNextCell.front();
        pathToNextCell.erase(pathToNextCell.begin());
        currentPosition = calcNextCell(currentPosition, nextStep);
        decreaseBattery();
        remainingSteps--;
        return nextStep;
    }

    return Step::Stay;
}

Step MyAlgorithm::handleCleaning() {
    if (dynamicMap.find(currentPosition) == dynamicMap.end()) {
        throw std::runtime_error("Cleaning an unexplored cell");
    }
    
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
        }
    } else {
        throw std::runtime_error("Cleaning an Invalid cell");
    }

    return Step::Stay;
}

void MyAlgorithm::executeStep(Step step) {
    switch (step) {
        case Step::North:
            currentPosition.y++;
            break;
        case Step::East:
            currentPosition.x++;
            break;
        case Step::South:
            currentPosition.y--;
            break;
        case Step::West:
            currentPosition.x--;
            break;
        case Step::Stay:
            break;
        case Step::Finish:
            std::cout << "Simulation finished!" << std::endl;
            break;
    }
}

Step MyAlgorithm::nextStep() {
    try {
        if (!initialized) {
            if (!initialize()) {
                throw std::runtime_error("No paths available from docking station");
            }
            initialized = true;
        }

        // Ensure we have enough steps to return to the docking station
        if (remainingSteps <= int(findPathToDocking().size())) {
            // If we're out of steps we finish the algorithm
            return handleDockingFinish();
        }

        if (getBatteryState() <= int(findPathToDocking().size())) {
            return handleDockingRecharge();
        }

        if (cleaning) {
            return handleCleaning();
        }

        pathToNextCell.clear();
        return handleWalkingToNextCell();

    } catch (const std::exception& e) {
        std::cerr << "Error in nextStep: " << e.what() << std::endl;
        return Step::Stay; // Default to stay in case of an error
    }
}

Direction MyAlgorithm::convertStepToDirection(Step step) {
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
                int returnDistance = bfsToDocking(next).size();

                if (newDistance + returnDistance <= maxLength) {
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
        if (current == dockingStation) {
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

std::vector<Step> MyAlgorithm::generatePath() {
    int limit = std::min(remainingSteps, getBatteryState());
    return bfs(currentPosition, limit);
}
