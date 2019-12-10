#include "Graph.h"
#include "DataReader.h"

using namespace std;


int main() {

    // Test with known graph
    Graph g = Graph();

    g.addVertice(1);
    g.addVertice(2);
    g.addVertice(3);
    g.addVertice(4);

    g.addEdge(1, 2, 0.6);
    g.addEdge(1, 3, -0.3);
    g.addEdge(1, 4, -0.5);
    g.addEdge(2, 3, 0.5);
    g.addEdge(2, 4, -1);
    g.addEdge(3, 4, 0.3);

    cout << "G ---------> " << endl;
    g.printVertices();
    g.printEdges();
    vector<vector<int>> clusters = g.greedyCC();
    cout << "+--------------------------+ "<< endl;

    Graph::printClusters(clusters);


    /*
    // Test with a random graph
    Graph test = Graph::generateRandomGraph(6);
    cout << "Vertices : ";
    test.printVertices();
    test.printEdges();
    vector<vector<int>> clusters = test.greedyCC();

    Graph::printClusters(clusters);


    // Test with real data
    /*
    vector<tuple<int, int, float>> data = DataReader::read(10, "HUMAN", "DANRE");

    Graph g = Graph::generateGraph(data);
    cout << "Vertices : ";
    g.printVertices();
    g.printEdges();
    vector<vector<int>> clusters = g.greedyCC();
    Graph::printClusters(clusters);
    */
    return 0;
}