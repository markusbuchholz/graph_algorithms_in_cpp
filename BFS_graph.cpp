#include <iostream>
#include <list>
#include <queue>
#include <algorithm>

//--------------------------------------------------------------------------------

class Graph
{

private:
    int V;

    std::list<int> *adj;

public:
    Graph(int v) : V(v)
    {

        std::cout << "initialization of graph with number of vertex equals: " << v << std::endl;

        adj = new std::list<int>[V];
    }

    void addEdge(int vStart, int vEnd);
    void traverseBFS(int vStart, std::vector<bool> &visited);
    void printBFS(int vStart);
};

//--------------------------------------------------------------------------------

void Graph::addEdge(int vStart, int vEnd)
{

    this->adj[vStart].push_back(vEnd);
}

//--------------------------------------------------------------------------------

void Graph::traverseBFS(int vStart, std::vector<bool> &visited)
{

    std::queue<int> queue;
    visited[vStart] = true;
    queue.push(vStart);

    while (std::count(visited.begin(), visited.end(), true) < this->V)
    {

        int node = queue.front();
        queue.pop();
        std::cout << " ==== visited node ==== : " << node << std::endl;

        for (auto i = adj[node].begin(); i != adj[node].end(); i++)
        {

            if (visited[*i] != true)
            {

                queue.push(*i);
                visited[*i] = true;
                std::cout << "adjacent edge : " << *i << std::endl;
            }
        }
    }
}

//--------------------------------------------------------------------------------

void Graph::printBFS(int vStart)
{

    std::vector<bool> visited; // = new std::vector<bool>[this->V];

    for (int i = 0; i < this->V; i++)
    {

        visited.push_back(false);
    }

    Graph::traverseBFS(vStart, visited);
}

//--------------------------------------------------------------------------------

int main()
{

    Graph g(6);


    // Edges for node 0
    g.addEdge(0, 1);
    g.addEdge(0, 2);

    // Edges for node 1
    g.addEdge(1, 0);
    g.addEdge(1, 3);
    g.addEdge(1, 4);

    // Edges for node 2
    g.addEdge(2, 0);
    g.addEdge(2, 4);

    // Edges for node 3
    g.addEdge(3, 1);
    g.addEdge(3, 4);
    g.addEdge(3, 5);

    // Edges for node 4
    g.addEdge(4, 1);
    g.addEdge(4, 2);
    g.addEdge(4, 3);
    g.addEdge(4, 5);

    // Edges for node 5
    g.addEdge(5, 3);
    g.addEdge(5, 4);

    
    g.printBFS(2);
}