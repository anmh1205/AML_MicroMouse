#include "AML_Maze.h"

Node *AML_Maze_NewNode(short i, short j)
{
    Node * ThisNode;
    short HalfSize;

    ThisNode = (Node *) malloc(sizeof(Node));
    HalfSize = MAZESIZE / 2;

    NODE_ROW = i;
    NODE_COL = j;
    NODE_VISITED = FALSE;
    
    // initializing the flood value
    if (i < HalfSize && j < HalfSize)
    {
        NODE_FLOODVAL = (HalfSize - 1 - i) + (HalfSize - 1 - j);
    }
    else if (i < HalfSize && j >= HalfSize)
    {
        NODE_FLOODVAL = (HalfSize - 1 - i) + (j - HalfSize);
    }
    else if (i >= HalfSize && j < HalfSize)
    {
        NODE_FLOODVAL = (i - HalfSize) + (HalfSize - 1 - j);
    }
    else if (i >= HalfSize && j >= HalfSize)
    {
        NODE_FLOODVAL = (i - HalfSize) + (j - HalfSize);
    }

    return ThisNode;
}


Maze *AML_Maze_NewMaze()
{
    Maze *ThisMap;
    short i, j;

    ThisMap = (Maze *) malloc(sizeof(Maze));

    // initialize all node in the map
    for (i = 0; i < MAZESIZE; i++)
    {
        for (j = 0; j < MAZESIZE; j++)
        {
            NODE_MAP[i][j] = AML_Maze_NewNode(i, j);
        }
    }

    // initialize all the neighbors
    for (i = 0; i < MAZESIZE; i++)
    {
        for (j = 0; j < MAZESIZE; j++)
        {
            NODE_MAPIJ->left = (j == 0) ? NULL : NODE_MAP[i][j - 1];
            NODE_MAPIJ->right = (j == MAZESIZE - 1) ? NULL : NODE_MAP[i][j + 1];
            NODE_MAPIJ->up = (i == 0) ? NULL : NODE_MAP[i - 1][j];
            NODE_MAPIJ->down = (i == MAZESIZE - 1) ? NULL : NODE_MAP[i + 1][j];
        }
    }
    
    return ThisMap;
}

