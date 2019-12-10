//
// Created by Sandra on 23/10/2019.
//

#ifndef START_GRAPH_H
#define START_GRAPH_H

#include <tuple>
#include <vector>
#include <list>
#include <iostream>
#include <cstring>
#include <queue>

using namespace std;

class Graph {
private:
    vector<int> vertices;
    vector<tuple<int, int, float>> edges;

public:
    // Constructor
    explicit Graph();

    // Getters & setters
    int getIndexOfAVertice(int);
    float getCapacityofAnEdge(int, int);
    vector<tuple<int, int, float>> getEdgesOfAVertice(int);
    bool isAnEdgeFromTo(int, int);

    list<int> getAdjacents(int);
    Graph getComponent(const bool*);
    Graph getPositiveGraph();

    float getVerticeTotalWeight(int);
    float getTotalWeight();

    void setCapacityofAnEdge(int, int, float);

    // Adding & deleting
    void addVertice(int);
    void deleteVertice(int);
    void addEdge(int, int, float);
    void addEdges(vector<tuple<int, int, float>>);
    void deleteEdge(int, int);
    void deleteEdgesOfAVertice(int);

    // Operations on graph

    // To check connection
    void DFS(int v, bool visited[]);
    bool* cleanVisited();
    void checkConnection(int v, vector<bool *> *components);

    // To get clusters
    vector<vector<int>> greedyCC();
    vector<int> findDensestSubgraph(float);

    // Operations on directed graph

    // To get a min cut
    Graph computeHfromG(float, float);
    bool bfs(Graph, int, int, int[]);
    void dfs(Graph, int, bool[]);
    Graph getMinCut(Graph g, int s, int t);

    // To cut then remove processed data
    static vector<int> doCut(Graph);
    Graph removeVertices(const vector<int>& verticesToRemove);

    // Displaying data
    void printAdjacents(int);
    void printEdges();
    void printVertices();
    void printComponents(const vector<bool *>&);
    static void printCluster(vector<int>&);
    static void printClusters(vector<vector<int>>&);

    // Generating graphs
    static Graph generateRandomGraph(int);
    static Graph generateGraph(vector<tuple<int, int, float>>);

};


#endif //START_GRAPH_H
