#include "my_battery_meter.h"

MyBatteryMeter::MyBatteryMeter(int batteryLevel, int maxBattery) {
    this->batteryLevel = batteryLevel;
    this->maxBattery = maxBattery;
}

void MyBatteryMeter::decreaseBattery() {
    if (batteryLevel > 0) {
        batteryLevel--;
    }
}

void MyBatteryMeter::chargeBattery() {
    if (batteryLevel + (maxBattery / 20) > maxBattery) {
        batteryLevel = maxBattery;
    }
    else {
        batteryLevel += (maxBattery / 20);
    }
}