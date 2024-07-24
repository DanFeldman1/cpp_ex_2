# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++14 -Wall -I.

# Source files
SOURCES = main.cpp my_algorithm.cpp my_battery_meter.cpp my_dirt_sensor.cpp my_house.cpp my_simulator.cpp my_wall_sensor.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
TARGET = myrobot

# Default rule
all: $(TARGET)

# Rule to create the target executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@

# Rule to compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJECTS) $(TARGET)

# Phony targets
.PHONY: all clean
