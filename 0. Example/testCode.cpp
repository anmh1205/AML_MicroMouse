#include "stdio.h"

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3
#define UNKNOWN 4
#define FLOOD_ONE_CELL 9050

// define structures needed for flood fill
struct cell_info
{
    // variables for north,east,south,west walls
    bool walls[4];
    bool visited;
};

// struct to hold cell info
struct wall_maze
{
    struct cell_info cells[16][16];
};

// struct to hold distance info
struct dist_maze
{
    int distance[16][16];
};

// struct to hold coordinates
struct coor
{
    int x;
    int y;
};

// stack for iterative floodfill
struct stack
{
    struct coor array[256];
    int index;
};

struct dist_maze distances;
struct wall_maze cell_walls_info;
struct stack update_stack;
struct stack move_queue;
struct coor target;

// Constructor for initializing coordinates
void init_coor(struct coor *c, int x, int y)
{
    c->x = x;
    c->y = y;
}

void printDistance(struct dist_maze *dm)
{
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            printf("%d  ", dm->distance[i][j]);
        }

        printf("\n");
    }
}

void printWall(struct wall_maze *wm)
{

    for (int j = 15; j >= 0; j--)
    {
        for (int i = 0; i < 16; i++)
        {
            if (wm->cells[i][j].walls[WEST])
            {
                colum();
            }
            else
            {
                printf(" ");
            }

            if (wm->cells[i][j].walls[NORTH])
            {
                cellUp();
            }
            else
            {
                printf(" ");
            }

            if (wm->cells[i][j].walls[SOUTH])
            {
                cellDown();
            }
            else
            {
                printf(" ");
            }

            if (wm->cells[i][j].walls[EAST])
            {
                colum();
            }
            else
            {
                printf(" ");
            }
        }
        printf("\n");
    }
}
void init_distance_maze(struct dist_maze *dm, struct coor *c, int center)
{
    // If we are trying to get to the center
    // Initialize the distance maze with the center four cells as zero
    if (center == 1)
    {
        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < 16; j++)
            {
                if (i <= 7 && j <= 7)
                    dm->distance[i][j] = ((7 - i) + (7 - j));
                if (i <= 7 && j > 7)
                    dm->distance[i][j] = ((7 - i) + (j - 8));
                if (i > 7 && j <= 7)
                    dm->distance[i][j] = ((i - 8) + (7 - j));
                if (i > 7 && j > 7)
                    dm->distance[i][j] = ((i - 8) + (j - 8));
            }
        }
    }
    // Generalized floodfill for any other cell
    else
    {
        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < 16; j++)
            {
                // get the distance to target cell
                int x = i - c->x;
                if(x<0) x = -x;
                int y = j - c->y;
                if (y < 0)
                    y = -y;
                dm->distance[i][j] = x + y;
            }
        }
    }

}

void cellUp()
{
    printf("+");
}

void cellDown()
{
    printf("_");
}

void colum()
{
    printf("|");
}

// Initialize all cells to unvisited
void init_wall_maze(struct wall_maze *wm)
{
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            wm->cells[i][j].walls[NORTH] = 0;
            wm->cells[i][j].walls[EAST] = 0;
            wm->cells[i][j].walls[SOUTH] = 0;
            wm->cells[i][j].walls[WEST] = 0;
            wm->cells[i][j].visited = 0;
            // put the walls surrounding the maze
            if (i == 0)
                wm->cells[i][j].walls[WEST] = 1;
            if (j == 0)
                wm->cells[i][j].walls[SOUTH] = 1;
            if (i == 15)
                wm->cells[i][j].walls[EAST] = 1;
            if (j == 15)
                wm->cells[i][j].walls[NORTH] = 1;
        }
    }

    cell_walls_info.cells[0][0].walls[EAST] = 1;
    cell_walls_info.cells[0][0].walls[SOUTH] = 1;
    cell_walls_info.cells[0][0].walls[WEST] = 1;

}

int main()
{
    init_coor(&target, 8, 7);
    // init_distance_maze(&distances, &target, 0);

    init_wall_maze(&cell_walls_info);

    struct coor c;
    init_coor(&c, 0, 0);

}