#ifndef MY_BATTERY_METER_H
#define MY_BATTERY_METER_H

#include "battery_meter.h"

class MyBatteryMeter : public BatteryMeter {
public:
    MyBatteryMeter();
    std::size_t getBatteryState() const override;

private:
    int batteryLevel;
};

#endif // MY_BATTERY_METER_H
