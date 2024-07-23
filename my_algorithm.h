#ifndef MY_ALGORITHM_H
#define MY_ALGORITHM_H

#include "abstract_algorithm.h"
#include "my_wall_sensor.h"
#include "my_dirt_sensor.h"
#include "my_battery_meter.h"
#include <unordered_map>
#include <vector>
#include <memory>

struct Position {
    int x, y;
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// Custom hash function for Position
struct PositionHash {
    std::size_t operator()(const Position& pos) const {
        return std::hash<int>()(pos.x) ^ std::hash<int>()(pos.y);
    }
};


// Custom hash function for Step
struct StepHash {
    std::size_t operator()(const Step& step) const {
        return std::hash<int>()(static_cast<int>(step));
    }
};

class MyAlgorithm : public AbstractAlgorithm {
public:
    MyAlgorithm();
    void setMaxSteps(std::size_t maxSteps) override;
    void setWallsSensor(const WallsSensor& sensor) override;
    void setDirtSensor(const DirtSensor& sensor) override;
    void setBatteryMeter(const BatteryMeter& meter) override;
    void setMaxBattery(int maxBattery);
    Step nextStep() override;

private:
    void initialize();
    Step exploreAndDecide();
    std::vector<Step> findPathToDocking();
    // chat calc is short for calculate btw 
    Position calcNextCell(Position current, Step step);
    Step oppositeOf(Step step);
    Step argmax(const std::unordered_map<Step, int, StepHash>& dict);
    std::vector<Step> bfs(const Position& start, int maxLength);
    std::vector<Step> bfsToDocking(const Position& start);
    Direction convertStepToDirection(Step step);

    // Utilzing the sensors
    bool isWall(Step step);
    int getBatteryState();
    void chargeBattery();
    int getDirtLevel();
    void decreaseBattery();
    
    
    
    
    Position currentPosition;
    Position dockingStation;
    int maxBattery;
    int remainingSteps;

    bool initialized;

    bool returningToCharge;
    bool returningToFinish;
    bool walkingToNextCell;
    bool cleaning;
    bool charging;

    int chargingSteps;
    Step lastExploredDirection;
    std::vector<Step> pathToDock;
    std::vector<Step> directions;
    std::vector<Step> walls;
    std::vector<Step> notWalls;
    std::vector<Step> pathToNextCell;
    size_t explorationIndex;

    std::unordered_map<Position, char, PositionHash> dynamicMap;

    std::unique_ptr<MyWallSensor> wallsSensor;
    std::unique_ptr<MyDirtSensor> dirtSensor;
    std::unique_ptr<MyBatteryMeter> batteryMeter;
};

#endif // MY_ALGORITHM_H
