#ifndef MY_BATTERY_METER_H
#define MY_BATTERY_METER_H

#include "battery_meter.h"

class MyBatteryMeter : public BatteryMeter {
public:
    MyBatteryMeter(int batteryLevel, int maxBattery);
    std::size_t getBatteryState() const override { return batteryLevel;};
    std::size_t getMaxBattery() const { return maxBattery;};
    void decreaseBattery();
    void chargeBattery();

private:
    int batteryLevel;
    int maxBattery;
};

#endif // MY_BATTERY_METER_H
