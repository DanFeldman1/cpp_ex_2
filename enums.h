#ifndef ROBOT_ENUMS_H_
#define ROBOT_ENUMS_H_

enum class Direction { North, East, South, West };
enum class Step { North, East, South, West, Stay, Finish };
enum class CellType { WALL, EMPTY, DOCK, DIRT };

#endif  // ROBOT_ENUMS_H_
