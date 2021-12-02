//dot a_star_graph.dot -Tpdf > a_star_graph.pdf
#include <iostream>
#include <vector>
#include <list>
#include <limits>
#include <set>
#include <tuple>
#include <map>

int INF = std::numeric_limits<int>::max();

class Graph
{

private:
    int V;

    std::list<std::pair<int, int>> *adj;

public:
    Graph(int v) : V(v)
    {

        std::cout << "initialization of graph for A* ..." << std::endl;

        this->V = v;

        this->adj = new std::list<std::pair<int, int>>[this->V];
    }
    void addEdge(int vStart, int vEnd, int cost);
    void Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<int> &heuristic);
    void computeStarA(int vStart, int vGoal, std::vector<int> heuristic);
};

//-------------------------------------------------------------------------------

void Graph::addEdge(int vStart, int vEnd, int cost)
{

    this->adj[vStart].push_back(std::make_pair(cost, vEnd));
}

//-------------------------------------------------------------------------------
/*
    g = min cost from start vertex to this vertex (cost is changing depending which vertexes have benn visited)
    h = heuristic (eg Manhattan distance) - constance
    (FX) f = g + h
    next step: min f

    */

void Graph::Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<int> &heuristic)
{

    std::vector<std::tuple<int, int, int>> path;
    std::vector<int> functionFX(V, INF);

    std::set<std::pair<int, int>> AStar_set;

    functionFX[vStart] = 0 + heuristic[vStart];

    AStar_set.insert(std::make_pair(functionFX[vStart], vStart));

    while ((*(AStar_set.begin())).second != vGoal)
    {

        std::pair<int, int> nodeMin = *(AStar_set.begin());

        int nodeGraph = nodeMin.second;

        AStar_set.erase(AStar_set.begin());

        visited[nodeGraph] = true;

        //traverse list for certein vertex

        for (auto i = adj[nodeMin.second].begin(); i != adj[nodeMin.second].end(); i++)
        {

            int nodeGraph_i = (*i).second;
            int nodeGraph_i_functionFX = (*i).first + heuristic[(*i).second];

            //check the cost - functionFX for each neighbors of current vertex
            if (visited[nodeGraph_i] != true)
            {

                if (functionFX[nodeGraph_i] > nodeGraph_i_functionFX)
                {
                    // Remove the current distance if it is in the set
                    if (functionFX[nodeGraph_i] != INF)
                    {
                        AStar_set.erase(AStar_set.find(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i)));
                    }

                    // Update the distance
                    functionFX[nodeGraph_i] = nodeGraph_i_functionFX;
                    AStar_set.insert(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i));

                    path.push_back(std::make_tuple(functionFX[nodeGraph_i], nodeGraph_i, nodeGraph));
                }
            }
        }
    }

    //---------------------------------------------------------------------

    std::multiset<std::tuple<int, int>> init_mSet;
    init_mSet.insert(std::make_tuple(0, 0));
    std::vector<std::multiset<std::tuple<int, int>>> routePath(V, init_mSet);

    for (int pathV = 1; pathV < V; pathV++)
    {

        std::multiset<std::tuple<int, int>> to_routePath;

        for (auto &ii : path)
        {

            int vertexV = std::get<1>(ii);

            if (pathV == vertexV)
            {

                to_routePath.insert(std::make_tuple(std::get<0>(ii), std::get<2>(ii))); //  Path_FX : Xx_xX, previous :Xx_xX
            }
        }

        routePath[pathV] = to_routePath;
    }

    //---------------------------------------------------------------------

    int previous = vGoal;
    std::vector<int> optimalPath;
    optimalPath.push_back(vGoal);

    while (previous != 0)
    {

        std::set<std::tuple<int, int>> minFx;

        for (auto &ii : routePath[previous])
        {

            minFx.insert(std::make_tuple(std::get<0>(ii), std::get<1>(ii)));
        }

        auto it = minFx.begin();
        previous = std::get<1>(*it);

        int min_path_i = std::get<0>(*it);

        optimalPath.push_back(previous);
    }

    //======= PRINT OPTIMAL PATH ===========

    std::cout << "Optimal A* path for given graph : " << std::endl;
    for (auto &ii : optimalPath)
    {

        std::cout << ii << "--";
    }

    std::cout << "\n";

    //======================================
}

//-------------------------------------------------------------------------------

void Graph::computeStarA(int vStart, int vGoal, std::vector<int> heuristic)
{

    std::vector<bool> visited(this->V, false);

    Graph::Astar(vStart, vGoal, visited, heuristic);
}

//-------------------------------------------------------------------------------

int main()
{

    Graph g(7);
    std::vector<int> heuristic{10, 8, 9, 5, 6, 3, 0};

    // Edges for node 0
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 4);

    // Edges for node 1
    g.addEdge(1, 0, 2);
    g.addEdge(1, 3, 1);
    g.addEdge(1, 4, 5);

    // Edges for node 2
    g.addEdge(2, 0, 4);
    g.addEdge(2, 4, 3);

    // Edges for node 3
    g.addEdge(3, 1, 1);
    g.addEdge(3, 4, 4);
    g.addEdge(3, 5, 2);
    g.addEdge(3, 6, 20);

    // Edges for node 4
    g.addEdge(4, 1, 5);
    g.addEdge(4, 2, 3);
    g.addEdge(4, 3, 4);
    g.addEdge(4, 5, 5);

    // Edges for node 5
    g.addEdge(5, 3, 2);
    g.addEdge(5, 4, 5);
    g.addEdge(5, 6, 2);

    // Edges for node 6
    g.addEdge(6, 3, 20);
    g.addEdge(6, 5, 2);

    g.computeStarA(0, 6, heuristic);
}