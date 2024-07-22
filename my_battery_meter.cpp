#include "my_battery_meter.h"

MyBatteryMeter::MyBatteryMeter(int batteryLevel) : batteryLevel(batteryLevel) {}

void MyBatteryMeter::decreaseBattery() {
    if (batteryLevel > 0) {
        batteryLevel--;
    }
}