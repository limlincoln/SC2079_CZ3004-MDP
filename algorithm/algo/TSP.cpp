#include <vector>
#include <set>
#include <queue>
#include <map>

using namespace std;

// Define Constants
#define TURNING_RADIUS 5
#define NORMAL_SPEED 1
#define TURNING_SPEED 2
#define SCALING_FACTOR 3
// Class TSP
class TSP
{

private:
public:
    TSP(/* args */);
    ~TSP();
    vector<Coordinate> aStar(vector<vector<int>> &grid, Coordinate start, Coordinate goal);
    int costFunction(Coordinate current, Coordinate neighbour);
    int heuristicFunction(Coordinate current, Coordinate goal);
    vector<Coordinate> get_neighbours(vector<vector<int>> &grid, Coordinate current);
    bool is_walkable(vector<vector<int>> &grid, Coordinate current);
};

// Define the coordinates of the grid
struct Coordinate
{
    int x;
    int y;
    float orientation;

    bool operator==(Coordinate &c1)
    {
        if (x == c1.x && y == c1.y)
            return true;
        return false;
    };

    bool operator!=(Coordinate &c1)
    {
        if (x == c1.x && y == c1.y)
            return false;
        return true;
    };
};

struct Obstacle
{
    int x;
    int y;
    float orientation;
    int ImageY;
    int ImageX;
};
int TSP::costFunction(Coordinate current, Coordinate neighbour)
{
    int cost = NORMAL_SPEED;
    if (current.x > neighbour.x || current.x < neighbour.x)
    {
        cost += (TURNING_RADIUS + TURNING_SPEED);
    };

    return cost;
};

// using Manhattan distance
int TSP::heuristicFunction(Coordinate current, Coordinate goal)
{
    int x = abs(current.x - goal.x);
    int y = abs(current.y - goal.y);
    return sqrt(pow(x, 2) + pow(y, 2));
};

// get neighbours from current node
vector<Coordinate> TSP::get_neighbours(vector<vector<int>> &grid, Coordinate current)
{
    vector<Coordinate> neighbours;
    vector<Coordinate> straight;
    vector<Coordinate> turn;
    int straightDist = 10 * SCALING_FACTOR;
    straight = {{current.x, current.y + straightDist}, {current.x, current.y - straightDist}};
    for (auto c : straight)
    {
        if (is_walkable(grid, c))
            neighbours.push_back(c);
    }
    turn = {}
};
// Define the search algorithm, such as Dijkstra's or A*
vector<Coordinate> TSP::aStar(vector<vector<int>> &grid, Coordinate start, Coordinate goal)
{

    // Implement the search algorithm here
    priority_queue<pair<int, Coordinate>, vector<pair<int, Coordinate>>, greater<pair<int, Coordinate>>> pq;
    set<Coordinate> visited;
    map<Coordinate, Coordinate> parent;
    vector<Coordinate> path;

    pq.push({0, start});
    visited.insert(start);
    while (!pq.empty())
    {
        auto current = pq.top().second;
        pq.pop();
        if (current == goal)
        {
            while (current != start)
            {
                path.push_back(current);
                current = parent[current];
            }

            // add start to the path;
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        };

        // loop through the neighbours of the current node
        for (auto neighbour : get_neighbours(grid, current))
        {
            if (visited.find(neighbour) != visited.end() || !is_walkable(grid, neighbour))
            {
                continue;
            }

            int cost = costFunction(current, neighbour) + heuristicFunction(current, goal);

            pq.push({cost, neighbour});

            visited.insert(neighbour);
            parent[neighbour] = current;
        };
    };
    // Return the shortest path from start to goal
    return {};
};

vector<Coordinate> path_planning(vector<vector<int>> &grid, Coordinate start, vector<Obstacle> &obstacles)
{
    vector<Coordinate> path;
    set<Obstacle> visited;
    path.push_back(start);

    for (auto obstacle : obstacles)
    {
        Coordinate goal = {obstacle.ImageX, obstacle.ImageY, obstacle.orientation};
        auto shortest_path = aStar(grid, path.back(), goal);
        path.insert(path.end(), shortest_path.begin(), shortest_path.end() - 1);
        visited.insert(obstacle);
    };

    // auto shortest_path = search_algorithm(grid, path.back(), start);
    // path.insert(path.end(), shortest_path.begin(), shortest_path.end());
    return path;
};