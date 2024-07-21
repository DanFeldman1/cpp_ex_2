#include "my_battery_meter.h"

MyBatteryMeter::MyBatteryMeter(int batteryLevel) : batteryLevel(batteryLevel) {}

std::size_t MyBatteryMeter::getBatteryState() const {
    return batteryLevel;
}

void MyBatteryMeter::decreaseBattery() {
    if (batteryLevel > 0) {
        batteryLevel--;
    }
}