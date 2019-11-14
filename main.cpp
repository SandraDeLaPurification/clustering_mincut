#include "Graph.h"
using namespace std;


int main() {
    Graph g(4);

    // Adding edges
    g.addEdge(1, 2, 0.6);
    g.addEdge(1, 3, -0.3);
    g.addEdge(1, 4, -0.5);
    g.addEdge(2, 3, 0.5);
    g.addEdge(2, 4, -1);
    g.addEdge(3, 4, 0.3);

    // Testing if g is a connected graph
    Graph positive = g.getPositiveGraph();
    if(positive.isConnected())
        cout << "G is a connected graph" << endl;
    else
        cout << "G is not a connected graph" << endl;

    // Get positive edges and compute H from G+
    Graph h = positive.computeHfromG(positive.getTotalWeight(), positive.getTotalWeight() / 2);
    h.printEdges();

    // get a min cut of h
    Graph minCut = h.getMinCut(h, 0, 5);
    cout << minCut.getTotalWeight() << " >= " << h.getTotalWeight() << " ?" << endl;

    // Cut the edges that are not from S to a vertice (except T)
    Graph cutedGraph = h.doCut(minCut);
    cout << "+--------------------------+" << endl;

    // Print the bestX
    cutedGraph.printEdges();
    return 0;
}