//
// Created by Sandra on 23/10/2019.
//

#ifndef START_GRAPH_H
#define START_GRAPH_H

#include <tuple>
#include <vector>
#include <list>
#include <iostream>

using namespace std;

class Graph {
private:
    int vertices;
    vector<tuple<int, int, float>> edges;

public:
    explicit Graph(int);
    int getVertices();
    vector<tuple<int, int, float>> getEdges();
    list<int> getAdjacents(int);

    void addEdge(int, int, float);
    void deleteEdge(int, int);
    void DFSUtil(int v, bool visited[]);
    bool* DFS(int v);

    bool isConnected();

    void printAdjacents(int);
    void printEdges();

    Graph getPositiveGraph();

    vector<Graph> greedyCC();

    Graph findDensestSubgraph(float);
    //vector<vector<int>> findDensestSubgraph(int);

    Graph computeHfromG(float, float);

    float getVerticeTotalWeight(int);
    float getTotalWeight();

    bool bfs(Graph, int, int, int[]);
    void dfs(Graph, int, bool[]);
    Graph getMinCut(Graph g, int s, int t);
    float getCapacityofAnEdge(int, int);
    void setCapacityofAnEdge(int, int, float);
    bool isAnEdgeFromTo(int, int);
    Graph doCut(Graph);
};


#endif //START_GRAPH_H
