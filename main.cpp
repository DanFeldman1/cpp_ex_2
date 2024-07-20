#include "my_wall_sensor.h"
#include "my_dirt_sensor.h"
#include "my_batter_meter.h"
#include "my_algorithm.h"

int main() {
    MyWallsSensor wallSensor;
    MyDirtSensor dirtSensor;
    MyBatteryMeter batteryMeter;
    MyAlgorithm algorithm;
    
    algorithm.setMaxSteps(1000);
    algorithm.setWallsSensor(wallSensor);
    algorithm.setDirtSensor(dirtSensor);
    algorithm.setBatteryMeter(batteryMeter);
    
    Step step;
    do {
        step = algorithm.nextStep();
        // Execute the step (for example, move the robot in the indicated direction)
        // This is just a placeholder to show the flow
    } while (step != Step::Finish);
    
    return 0;
}
