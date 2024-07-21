#include "my_battery_meter.h"

MyBatteryMeter::MyBatteryMeter() : batteryLevel(100) { // Assuming full battery level at start
}

std::size_t MyBatteryMeter::getBatteryState() const {
    return batteryLevel;
}
