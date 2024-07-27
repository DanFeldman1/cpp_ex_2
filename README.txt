
# MyAlgorithm Project

This project is a C++ implementation of an autonomous robot algorithm designed to navigate, clean, and return to its docking station within a specified environment. The robot uses sensors to detect walls, dirt, and manage its battery level to perform its cleaning task efficiently. This README provides an overview of the project structure, usage instructions, and an explanation of the algorithm.


# MAKE SURE THAT IN ORDER TO RUN
# HOUSE LAYOUT MUST END WITH AN EMPTY NEWLINE ***

## Project Structure

```
.
├── .vscode
├── src
│   ├── abstract_algorithm.h
│   ├── battery_meter.h
│   ├── dirt_sensor.h
│   ├── enums.h
│   ├── input1.txt
│   ├── input2.txt
│   ├── main.cpp
│   ├── Makefile
│   ├── my_algorithm.cpp
│   ├── my_algorithm.h
│   ├── my_battery_meter.cpp
│   ├── my_battery_meter.h
│   ├── my_dirt_sensor.cpp
│   ├── my_dirt_sensor.h
│   ├── my_house.cpp
│   ├── my_house.h
│   ├── my_simulator.cpp
│   ├── my_simulator.h
│   ├── my_wall_sensor.cpp
│   ├── my_wall_sensor.h
│   ├── output.txt
├── README.md
```

## Compilation

To compile the project, you can use the provided `Makefile` or a C++ compiler like `g++`. 

### Using Makefile

```sh
make
```

### Using g++

If you prefer using `g++`, here's an example command to compile all the source files:

```sh
g++ -std=c++17 -o myrobot src/main.cpp src/my_algorithm.cpp src/my_simulator.cpp src/my_battery_meter.cpp src/my_dirt_sensor.cpp src/my_wall_sensor.cpp src/my_house.cpp
```

## Usage

To run the program, you need to provide a house input file. The program will output the results to a file named `output_<house_input_file_name>.txt`.

```sh
./myrobot src/input1.txt
```

### Input File Format

The input file represents the house layout and must follow this format:

```
<house name / description>
MaxSteps = <NUM>
MaxBattery = <NUM>
Rows = <NUM>
Cols = <NUM>
<house layout> 
```

### Output File Format

The output file contains the results of the robot's cleaning process:

```
NumSteps = <NUMBER>
DirtLeft = <NUMBER>
Status = <FINISHED/WORKING/DEAD>
Steps:
<list of characters in one line, no spaces, from: NESWsF>
```

## Code Explanation

### `MyAlgorithm` Class

The `MyAlgorithm` class is the core of the robot's decision-making process. It implements the `AbstractAlgorithm` interface and contains the logic for navigating, cleaning, and recharging the robot.

#### Key Methods

- `setMaxSteps`: Sets the maximum number of steps the robot can take.
- `setWallsSensor`: Sets the wall sensor.
- `setDirtSensor`: Sets the dirt sensor.
- `setBatteryMeter`: Sets the battery meter.
- `initialize`: Initializes the algorithm, determining the initial positions and mapping the surroundings.
- `nextStep`: Determines the next step for the robot to take based on its current state and surroundings.
- `handleDockingFinish`: Handles the logic when the robot finishes its task and returns to the docking station.
- `handleDockingRecharge`: Handles the logic when the robot needs to recharge at the docking station.
- `handleCharging`: Handles the charging process at the docking station.
- `handleWalkingToNextCell`: Handles the logic for navigating to the next cell to clean.
- `handleCleaning`: Handles the cleaning process at the current cell.
- `bfs`: Breadth-First Search to find a path to the next unexplored or dirty cell within a specified distance.
- `bfsToDocking`: Breadth-First Search to find a path back to the docking station.
- `generatePath`: Generates a path to the next cell to clean, ensuring the robot can return to the docking station within the battery and step constraints.

### `MySimulator` Class

The `MySimulator` class manages the simulation, including reading the house input file, setting the algorithm, and running the simulation.

#### Key Methods

- `readHouseFile`: Reads the house layout from the input file.
- `setAlgorithm`: Sets the cleaning algorithm for the robot.
- `executeStep`: Executes a step for the robot based on the algorithm's decision.
- `run`: Runs the simulation, executing steps until the robot finishes or runs out of steps.

### Sensors

The sensors provide the robot with information about its environment:

- `MyWallSensor`: Detects walls around the robot.
- `MyDirtSensor`: Measures the dirt level of the current cell.
- `MyBatteryMeter`: Manages the robot's battery state.

### Enums

The `enums.h` file defines the enumeration types for directions and steps:

```cpp
enum class Direction { North, East, South, West };
enum class Step { North, East, South, West, Stay, Finish };
```

## Example

Here's an example of running the program:

1. Create an input file `src/input1.txt` with the following content:

```
Example House
MaxSteps = 100
MaxBattery = 20
Rows = 4
Cols = 4
WWWW
WD1W
W21W
WWWW
```

2. Run the program:

```sh
./myrobot src/input1.txt
```

3. Check the output file `output/output_input1.txt` for the results.

---

This project demonstrates a simple yet effective approach to autonomous robot navigation and cleaning. The algorithm ensures the robot efficiently cleans the house while managing its battery and steps to return to the docking station.
