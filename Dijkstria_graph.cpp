#include <iostream>
#include <limits>
#include <list>
#include <vector>
#include <algorithm>
#include <set>

int INF = std::numeric_limits<int>::max();

//------------------------------------------------------------

class Graph
{

private:
    std::list<std::pair<int, int>> *adj;
    int V;

public:
    Graph(int v) : V(v)
    {

        std::cout << "Graph inititalization with : " << v << " nodes " << std::endl;

        this->adj = new std::list<std::pair<int, int>>[v];
    }

    void addEdge(int vStart, int vEnd, int cost);
    void Dijkstria(int vStart, std::vector<bool> visited);
    void computeShortestPath(int vStart);
};

//------------------------------------------------------------

void Graph::addEdge(int vStart, int vEnd, int cost)
{

    this->adj[vStart].push_back(std::make_pair(cost, vEnd));
}

//------------------------------------------------------------

void Graph::Dijkstria(int vStart, std::vector<bool> visited)
{

    std::vector<int> distanceX(V, INF);

    std::set<std::pair<int, int>> dijkstria_set;

    distanceX[vStart] = 0;

    dijkstria_set.insert(std::make_pair(distanceX[vStart], vStart));

    while (!dijkstria_set.empty())
    {

        std::pair<int, int> nodeMin = *(dijkstria_set.begin());

        int nodeGraph = nodeMin.second;

        dijkstria_set.erase(dijkstria_set.begin());

        visited[nodeGraph] = true;

        for (auto i = adj[nodeMin.second].begin(); i != adj[nodeMin.second].end(); i++)
        {

            int nodeGraph_i = (*i).second;
            int nodeGraph_i_const_cost = (*i).first;

            if (visited[nodeGraph_i] != true)
            {

                if (distanceX[nodeGraph_i] > distanceX[nodeGraph] + nodeGraph_i_const_cost)
                {

                    if (distanceX[nodeGraph_i] != INF)
                    {
                        dijkstria_set.erase(dijkstria_set.find(std::make_pair(distanceX[nodeGraph_i], nodeGraph_i)));
                    }

                    distanceX[nodeGraph_i] = (distanceX[nodeGraph] + nodeGraph_i_const_cost);
                    dijkstria_set.insert(std::make_pair(distanceX[nodeGraph_i], nodeGraph_i));
                }
            }
        }
    }

    std::cout << "Minimum distances from node: " << vStart << std::endl;
    for (int i = 0; i < V; i++)
    {
        std::cout << "node : " << i << "  edge : " << distanceX[i] << std::endl;
    }
}

//------------------------------------------------------------

void Graph::computeShortestPath(int vStart)
{

    std::vector<bool> visited(V, false);

    Graph::Dijkstria(vStart, visited);
}

//------------------------------------------------------------

int main()
{

    Graph g(9);

    // Add node 0
    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);

    // Add node 1
    g.addEdge(1, 0, 4);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);

    // Add node 2
    g.addEdge(2, 1, 8);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(2, 3, 7);

    // Add node 3
    g.addEdge(3, 2, 7);
    g.addEdge(3, 5, 14);
    g.addEdge(3, 4, 9);

    // Add node 4
    g.addEdge(4, 3, 9);
    g.addEdge(4, 5, 10);

    // Add node 5
    g.addEdge(5, 6, 2);
    g.addEdge(5, 3, 14);
    g.addEdge(5, 4, 10);

    // Add node 6
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(6, 5, 2);

    // Add node 7
    g.addEdge(7, 0, 8);
    g.addEdge(7, 1, 11);
    g.addEdge(7, 8, 7);
    g.addEdge(7, 6, 1);

    // Add node 8
    g.addEdge(8, 2, 2);
    g.addEdge(8, 7, 7);
    g.addEdge(8, 6, 6);

    g.computeShortestPath(0);
}