#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;

/* Map class
   Inside the map class, define the mapWidth, mapHeight and grid as a 2D vector
*/
class Map {
    public: 
        const static int mapWidth = 6;
        const static int mapHeight = 5;
        vector<vector<int>> grid = {
            { 0, 1, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0 },
            { 0, 0, 0, 1, 1, 0 },
        };
        vector<vector<int>> heuristic = {
            { 9, 8, 7, 6, 5, 4 },
            { 8, 7, 6, 5, 4, 3 },
            { 7, 6, 5, 4, 3, 2 },
            { 6, 5, 4, 3, 2, 1 },
            { 5, 4, 3, 2, 1, 0 },
        };
};

/* Planner class
   Inside the Planner class, define the start, goal, cost, movements, and movements_arrows
   Note: The goal should be defined it terms of the mapWidth and mapHeight
*/
class Planner : Map {
    public:
        int start[2] = { 0, 0 };
        int goal[2] = { mapHeight - 1, mapWidth - 1 };
        int cost = 1;

        string movements_arrows[4] = { "^", "<", "v", ">" };

        vector<vector<int>> movements {
            { -1, 0 },
            { 0, -1 },
            { 1, 0 },
            { 0, 1 }
        };
};

// Template function to print 2D vectors of any  type
/* print2DVector function which will print 2D vectors of any data type
   Example
   
   Input: 
   vector<vector<int> > a{{ 1, 0 },{ 0, 1 }};
   print2DVector(a);
   vector<vector<string> > b{{ "a", "b" },{ "c", "d" }};
   print2DVector(b);
   
   Output:
   1 0
   0 1
   a b
   c d
*/
template <typename T>
void print2DVector(T Vec)
{
    for(int i = 0; i < Vec.size(); ++i) {
        for(int j = 0; j < Vec[0].size(); ++j) {
            cout << Vec[i][j] << ' ';
        }
        cout << endl;
    }
}

int manhattanDistance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

/* BFS: search function which will generate the expansion list ####*/
// Search function which generates the expansions
void search(Map map, Planner planner)
{
    // Create a closed 2 array filled with 0s and first element 1
    vector<vector<int>> closed(map.mapHeight, vector<int>(map.mapWidth));
    closed[planner.start[0]][planner.start[1]] = 1;

    // Create expand array filled with -1
    vector<vector<int>> expand(map.mapHeight, vector<int>(map.mapWidth, -1));

    vector<vector<int>> action(map.mapHeight, vector<int>(map.mapWidth, -1));
    action[planner.start[0]][planner.start[1]] = 1;

    // Defined the triplet values
    int x = planner.start[0];
    int y = planner.start[1];
    int g = 0;
    int orderOfVisit = 0;
    int direction = 0;
    int f = map.heuristic[x][y];

    // Store the expansions
    vector<vector<int>> open;
    open.push_back({f, g, x, y});

    // Flags
    bool found = false;
    bool resign = false;

    int x2;
    int y2;

    // While I am stil searching for the goal and the problem is solvable
    while(!found && !resign)
    {
        // Resign if no values in the open list and you can't expand anymore
        if(open.size() == 0)
        {
            resign = true;
            cout << "Failed to reach a goal" << endl;
        }
        else
        {
            // Keep expanding
            // Remove triplets from the open list
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            vector<int> next;
            // Stored the poped value into next
            next = open.back();
            open.pop_back();

            f = next[0];
            g = next[1];
            x = next[2];
            y = next[3];
            
            expand[x][y] = orderOfVisit;
            // cout << "[" << orderOfVisit << " , " << x << " , " << y << "]" << endl;

            orderOfVisit += 1;

            // Check if we reached the goal:
            if (x == planner.goal[0] && y == planner.goal[1])
            {
                found = true;
                // cout << "[" << g << " , " << x << " , " << y << "]" << endl;
            }
            else
            {
                // else expand new elements
                for(int i = 0; i < planner.movements.size(); i++)
                {
                    x2 = x + planner.movements[i][0];
                    y2 = y + planner.movements[i][1];
                    if(x2 >= 0 && x2 < map.grid.size() && y2 >= 0 && y2 < map.grid[0].size())
                    {
                        if(closed[x2][y2] == 0 && map.grid[x2][y2] == 0)
                        {
                            int g2 = g + planner.cost;
                            f = g2 + map.heuristic[x2][y2];
                            open.push_back({ f, g2, x2, y2 });
                            closed[x2][y2] = 1;
                            action[x2][y2] = i; 
                        }
                    }
                }
            }
        }
    }

    print2DVector(expand);

    vector<vector<string>> policy(map.mapHeight, vector<string>(map.mapWidth, "-"));

    x = planner.goal[0];
    y = planner.goal[1];
    policy[x][y] = '*';

    while(x != planner.start[0] || y != planner.start[1])
    {
        x2 = x - planner.movements[action[x][y]][0];
        y2 = y - planner.movements[action[x][y]][1];
        policy[x2][y2] = planner.movements_arrows[action[x][y]];
        x = x2;
        y = y2;
    }

    cout << endl; 
    print2DVector(policy);
}

int main()
{
    // Instantiate map and planner objects
    Map map;
    Planner planner;

    // Print classes variables
    // cout << "Map: " << endl;
    // print2DVector(map.grid);
    // cout << "Start: " << planner.start[0] << " , " << planner.start[1] << endl;
    // cout << "Goal: " << planner.goal[0] << " , " << planner.goal[1] << endl;
    // cout << "Cost: " << planner.cost << endl;
    // cout << "Robot Movements: " << planner.movements_arrows[0] << " , " << planner.movements_arrows[1] << " , " << planner.movements_arrows[2] << " , " << planner.movements_arrows[3] << endl;
    // cout << "Delta: " << endl;
    // print2DVector(planner.movements);

    // Search for the expansions
    search(map, planner);

    return 0;
}
/*
0 -1 13 17 -1 -1 
1 -1 10 14 18 -1 
2 -1 8 11 15 19 
3 -1 7 9 12 16 
4 5 6 -1 -1 20 

*/