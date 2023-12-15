#ifndef AML_Parameter_H
#define AML_Parameter_H

#include "stm32f4xx.h"

// store all the parameters

#define NO_RIGHT_WALL 175
#define NO_LEFT_WALL 175
#define NO_FRONT_WALL 90
#define NO_FRONT_LEFT_WALL 90
#define NO_FRONT_RIGHT_WALL 90

#define RIGHT_WALL 165
#define LEFT_WALL 165
#define FRONT_WALL 60
#define FRONT_LEFT_WALL 110
#define FRONT_RIGHT_WALL 140

#define FLOOD_NO_RIGHT_WALL 175
#define FLOOD_NO_LEFT_WALL 175
#define FLOOD_NO_FRONT_WALL 220
#define FLOOD_NO_FRONT_LEFT_WALL 90
#define FLOOD_NO_FRONT_RIGHT_WALL 90

#define FLOOD_RIGHT_WALL 165
#define FLOOD_LEFT_WALL 165
#define FLOOD_FRONT_WALL 140
#define FLOOD_FRONT_LEFT_WALL 110
#define FLOOD_FRONT_RIGHT_WALL 140

#define ENCODER_TICKS_ONE_CELL 1438
#define FLOOD_ENCODER_TICKS_ONE_CELL 1361

//////////////////////////////////

// competition maze parameters

#define WALL_IN_FRONT 115 // 47
#define WALL_IN_LEFT 125 // 121
#define WALL_IN_RIGHT 125 // 100
#define WALL_IN_FRONT_LEFT 80 // 60
#define WALL_IN_FRONT_RIGHT 80 // 50

#define WALL_NOT_IN_FRONT 160
#define WALL_NOT_IN_LEFT 160
#define WALL_NOT_IN_RIGHT 110 
#define WALL_NOT_IN_FRONT_LEFT 95 
#define WALL_NOT_IN_FRONT_RIGHT 85

#define MAZE_ENCODER_TICKS_ONE_CELL 1350  // Maze BK
// #define MAZE_ENCODER_TICKS_ONE_CELL 1400  // Maze at home

#define MAZE_FLOOD_ENCODER_TICKS_ONE_CELL 1361
#define MAZE_ENCODER_TICKS_FLAG 1000

//////////////////////////////////




#endif // AML_Parameter_H