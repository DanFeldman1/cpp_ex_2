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
    interruptedCleaningPosition({-1, -1}),
    maxBattery(0),
    remainingSteps(0),
    initialized(false),
    returningToDock(false),
    backtracking(false),
    cleaning(false),
    interruptedCleaning(false),
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
    interruptedCleaningPosition = {-1, -1};
    // batteryState = maxBattery; 
    returningToDock = false;
    backtracking = false;
    cleaning = false;
    interruptedCleaning = false;
    charging = false;
    chargingSteps = 0;
    directions = {Step::North, Step::South, Step::East, Step::West};
    initialized = false;
    explorationIndex = 0;
    walls.clear();
    notWalls.clear();

    for (Step step : directions) {
        if (isWall(step)) {
            walls.push_back(step);
            Position wallPos = calcNextCell(currentPosition, step);
            visited[wallPos] = -1;
            tempDict[step] = -1;
        } else {
            notWalls.push_back(step);
        }
    }
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

Step MyAlgorithm::exploreAndDecide() {
    while (explorationIndex < notWalls.size()) {
        Step step = notWalls[explorationIndex];
        Position newPos = calcNextCell(currentPosition, step);
        if (visited.find(newPos) == visited.end()) {  // If not already initialized
            lastExploredDirection = step;
            backtracking = true;
            return step;  // Move to this direction to explore
        }
        this->tempDict[step] = visited[newPos];
        explorationIndex++;
    }

    return argmax(this->tempDict);  // Move to the dirtiest direction
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
            returningToDock = true;
            cleaning = false;
            interruptedCleaning = false;
            if (currentPosition == dockingStation) {
                return Step::Finish;
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

        // Check if battery is low and initiate return to docking station if necessary
        if (getBatteryState() <= static_cast<int>(bfs(currentPosition).size())) {
            returningToDock = true;
            if (cleaning) {
                interruptedCleaning = true;
                interruptedCleaningPosition = currentPosition;
                cleaning = false;
            }
        }

        // Handle charging at the docking station
        if (charging) {
            if (chargingSteps < 20 && getBatteryState() < maxBattery) {
                chargingSteps++;
                chargeBattery();
                remainingSteps--;
                return Step::Stay;
            } else {
                charging = false;
                chargingSteps = 0;
                returningToDock = false;
            }
        }

        // Handle returning to the docking station for recharging
        if (returningToDock) {
            if (currentPosition == dockingStation) {
                returningToDock = false;
                charging = true;
                chargingSteps = 0;
                return Step::Stay; // Start charging at docking station
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

        // Resume interrupted cleaning
        if (interruptedCleaning) {
            if (interruptedCleaningPath.empty()) {
                interruptedCleaningPath = bfs(currentPosition);
            }

            if (!interruptedCleaningPath.empty()) {
                Step nextStep = interruptedCleaningPath.front();
                interruptedCleaningPath.erase(interruptedCleaningPath.begin());
                currentPosition = calcNextCell(currentPosition, nextStep);
                decreaseBattery();
                remainingSteps--;
                if (interruptedCleaningPath.empty()) {
                    interruptedCleaning = false;
                    cleaning = true;
                }
                return nextStep;
            }
        }

        // Handle cleaning
        if (cleaning) {
            if (visited[currentPosition] > 0) {
                visited[currentPosition]--;
                decreaseBattery();
                remainingSteps--;
                if (visited[currentPosition] == 0) {
                    cleaning = false; // Done cleaning this tile
                }
                return Step::Stay; // Continue cleaning
            }
            cleaning = false; // No more dirt to clean
        }

        // Handle backtracking
        if (backtracking) {
            backtracking = false;
            Step backDirection = oppositeOf(lastExploredDirection);
            currentPosition = calcNextCell(currentPosition, backDirection);
            decreaseBattery();
            remainingSteps--;
            return backDirection;
        }

        // Handle exploration and decision making
        Step choice = exploreAndDecide();
        Position newPos = calcNextCell(currentPosition, choice);

        // Simulate checking dirt level if moving to a new position
        if (visited.find(newPos) == visited.end()) {
            visited[newPos] = getDirtLevel();
        }

        currentPosition = newPos;

        if (visited[newPos] > 0) {
            cleaning = true; // Start cleaning if the tile has dirt
        }

        decreaseBattery();
        remainingSteps--;
        return choice;
    } catch (const std::exception& e) {
        std::cerr << "Error in nextStep: " << e.what() << std::endl;
        return Step::Stay; // Default to stay in case of an error
    }
}

Step MyAlgorithm::oppositeOf(Step dir) {
    switch (dir) {
        case Step::North:
            return Step::South;
        case Step::South:
            return Step::North;
        case Step::East:
            return Step::West;
        case Step::West:
            return Step::East;
        default:
            return Step::Stay;
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
            return;
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
            return path;  // Return the path to the first unexplored or dirty cell
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