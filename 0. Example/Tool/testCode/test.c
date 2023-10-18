#include "stdio.h"
#include "stdlib.h"
#include "AML_Maze.h"


struct Maze *ThisMap;

int main()
{
    ThisMap = AML_Maze_NewMaze();
}