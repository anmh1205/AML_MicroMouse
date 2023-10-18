// #pragma once
#ifndef AML_Maze_H
#define AML_Maze_H

#include "stdio.h"
#include "stdlib.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MAZESIZE 16

// Shortcut Constants
#define NODE_MAPIJ ThisMap->map[i][j]
#define NODE_MAP ThisMap->map
#define NODE_FLOODVAL ThisNode->floodval
#define NODE_ROW ThisNode->row
#define NODE_COL ThisNode->column
#define NODE_VISITED ThisNode->visited
#define NODE_LEFT ThisNode->left
#define NODE_RIGHT ThisNode->right
#define NODE_UP ThisNode->up
#define NODE_DOWN ThisNode->down

// Define Direction
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define UNKNOWN 4
#define FLOOD_ONE_CELL 9050

// Stack Constants
#define SPI 1 // Stack Pointer Index
#define SSI 0 // Stack Size Index
#define STACK_OFFSET 2
#define STACKSIZE 80

typedef struct Node
{

    /* data fields */
    short floodval;
    short row;
    short column;
    short visited;

    /* pointers to neighbors */
    struct Node *left;
    struct Node *right;
    struct Node *up;
    struct Node *down;

} Node;

typedef struct Maze
{
    Node *map[MAZESIZE][MAZESIZE];
} Maze;

typedef struct Stack
{

    short properties[STACK_OFFSET];
    Node *the_stack[STACKSIZE];

} Stack;


// Function Prototypes
Node *AML_Maze_NewNode(short i, short j);

// Maze Functions
Maze *AML_Maze_NewMaze();

#endif