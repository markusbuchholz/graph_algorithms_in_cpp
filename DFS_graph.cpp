#include <iostream>
#include <list>
#include <vector>

//--------------------------------------------------------------------------------


class Graph
{

private:
    int V;

    std::list<int> *adj;

public:
    Graph(int v) : V(v)
    {

        std::cout << "init of graph with size of : " << v << std::endl;
        this->adj = new std::list<int>[v];
    }

    void addEdge(int vStart, int vEnd);
    void traverseDFS (int vStart, bool *visited);
    void printDFS(int vStart);
};

//--------------------------------------------------------------------------------


void Graph::addEdge(int vStart, int vEnd)
{

    this->adj[vStart].push_back(vEnd);
}

void Graph::traverseDFS (int vStart, bool *visited)
{
    visited[vStart] = true;

    for (auto i = adj[vStart].begin(); i != adj[vStart].end(); i++)
    {

        if (visited[*i] == false)
        {

            std::cout << "from vertex : " << vStart << " to vertex : " << *i << std::endl;

            Graph::traverseDFS(*i, visited);
        }
    }
}


//--------------------------------------------------------------------------------


void Graph::printDFS(int vStart)
{

    bool *visited = new bool[this->V];

    for (int i = 0; i < this->V; i++)
    {

        visited[i] = false;
    }

    Graph::traverseDFS(vStart, visited);
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

    // Perform DFS and print result
    g.printDFS(0);
}
