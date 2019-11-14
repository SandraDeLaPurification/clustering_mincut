//
// Created by Sandra on 23/10/2019.
//

#include <cstring>
#include <queue>
#include "Graph.h"


Graph::Graph(int v) {
    this->vertices = v;
    this->edges = vector<tuple<int, int, float>>();
}

int Graph::getVertices() {
    return this->vertices;
}

vector<tuple<int, int, float>> Graph::getEdges() {
    return this->edges;
}

list<int> Graph::getAdjacents(int v) {
    list<int> adjacents;

    int index = 0;

    for(auto & edge : this->edges){
        if(v == get<0>(edge)){
            adjacents.push_back(get<1>(edge));
        }
        else if(v == get<1>(edge)){
            adjacents.push_back(get<0>(edge));
        }
    }
    adjacents.sort();
    adjacents.unique();
    return adjacents;
}


void Graph::addEdge(int u, int v, float capacity) {
    this->edges.emplace_back(make_tuple(u, v, capacity));
}

void Graph::DFSUtil(int v, bool *visited) {
    // Mark the current node as visited and
    // print it
    visited[v] = true;
    //cout << v << " ";

    // Recur for all the vertices adjacent
    // to this vertex
    list<int> adj = this->getAdjacents(v);
    list<int>::iterator i;
    for (i = adj.begin(); i != adj.end(); ++i)
        if (!visited[*i])
            DFSUtil(*i, visited);
}

bool* Graph::DFS(int v) {
    // Mark all the vertices as not visited
    bool *visited = new bool[this->vertices];
    for (int i = 0; i < this->vertices; i++)
        visited[i] = false;

    // Call the recursive helper function
    // to print DFS traversal
    DFSUtil(v, visited);
    return visited;
}

bool Graph::isConnected() {
    bool* visited = this->DFS(get<0>(this->edges[0]));
    for(int i = 1; i < this->vertices; i++)
        if(!visited[i])
            return false;
    return true;
}

void Graph::printAdjacents(int vertice) {
    list<int> voisins = this->getAdjacents(vertice);
    list<int>::iterator i;
    for (i = voisins.begin(); i != voisins.end(); ++i){
        printf("Neighbor %d : %d \n", vertice, *i);
    }
}

Graph Graph::getPositiveGraph() {
    Graph positive(this->vertices);
    for(auto & edge : this->edges)
        if(get<2>(edge) >= 0) {
            cout << get<0>(edge) << " to " << get<1>(edge) << " is a positive edge" << endl;
            positive.edges.emplace_back(edge);
        }
    return positive;
}

vector<Graph> Graph::greedyCC() {
    vector<Graph> result;
    Graph positive = this->getPositiveGraph();
    if(!positive.isConnected()){
        // TO DO : find components and for each greedyCC()
    }
    else{
        result.emplace_back(positive.findDensestSubgraph(0));
        // TO DO : P - X then P.greedy()
    }
    return result;
}

Graph Graph::findDensestSubgraph(float delta) {
    // Initialization
    float weight = getTotalWeight();
    float min = 0, max = weight, minG = 0;
    bool done = false;
    Graph bestX(0);

    cout << "Start with min = " << min << " & max = " << max << endl;
    while(!done){
        minG = min + (max - min) / 2;
        cout << "MinG = " << minG << endl;
        Graph c = getMinCut(this->computeHfromG(weight, minG), 0, 5);

        if(c.getTotalWeight() >= (float)this->vertices * weight) {
            cout << c.getTotalWeight() << " >= " << (float)this->vertices * weight << " so continue" << endl;
            max = minG;
        }
        else{
            cout << "We found a bestX" << endl;
            bestX = this->doCut(c);
            min = minG;
        }
        if(max - min < (float)delta)
            done = true;
    }
    return bestX;
}

Graph Graph::computeHfromG(float weight, float minG) {
    Graph h(this->vertices + 2);
    for(auto & edge : this->edges){
        h.addEdge(0, get<0>(edge), weight);
        h.addEdge(get<0>(edge), get<1>(edge), get<2>(edge));
        h.addEdge(get<1>(edge), get<0>(edge), get<2>(edge));
        h.addEdge(get<0>(edge), h.vertices - 1, weight + 2 * minG  - getVerticeTotalWeight(get<0>(edge)));
    }
    h.addEdge(0, h.vertices - 2, weight);
    h.addEdge(h.vertices - 2, h.vertices - 1, weight + 2 * minG - getVerticeTotalWeight(h.vertices - 2));
    return h;
}

float Graph::getVerticeTotalWeight(int v) {
    float weight = 0;
    for(const auto & edge : this->edges){
        if(v == get<0>(edge) || v == get<1>(edge))
            weight += get<2>(edge);
    }
    return weight;
}

float Graph::getTotalWeight() {
    float weight = 0;
    for(auto & edge : this->edges)
        weight += get<2>(edge);
    return weight;
}

void Graph::printEdges() {
    for(auto & edge : this->edges){
        cout << "Edge from " << get<0>(edge) << " to " << get<1>(edge) << " with a capacity of " << get<2>(edge) << endl;
    }
}


/* Returns true if there is a path from source 's' to sink 't' in
  residual graph. Also fills parent[] to store the path */
bool Graph::bfs(Graph rGraph, int s, int t, int parent[])
{
    // Create a visited array and mark all vertices as not visited
    bool visited[this->vertices];
    memset(visited, 0, sizeof(visited));

    // Create a queue, enqueue source vertex and mark source vertex
    // as visited
    queue <int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;

    // Standard BFS Loop
    while (!q.empty())
    {
        int u = q.front();
        q.pop();

        for (int v = 0; v < this->vertices; v++)
        {
            if (!visited[v] && rGraph.isAnEdgeFromTo(u, v))
            {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }

    // If we reached sink in BFS starting from source, then return
    // true, else false
    return visited[t];
}

void Graph::dfs(Graph rGraph, int s, bool visited[]) {
    visited[s] = true;
    for (int i = 0; i < this->vertices; i++)
        if (rGraph.isAnEdgeFromTo(s, i) && !visited[i])
            dfs(rGraph, i, visited);
}

// Prints the minimum s-t cut
Graph Graph::getMinCut(Graph g, int s, int t)
{
    int u, v;

    // Create a residual graph and fill the residual graph with
    // given capacities in the original graph as residual capacities
    // in residual graph
    Graph rGraph = g;

    int parent[this->vertices];  // This array is filled by BFS and to store path

    // Augment the flow while there is a path from source to sink
    while (bfs(rGraph, s, t, parent))
    {
        // Find minimum residual capacity of the edges along the
        // path filled by BFS. Or we can say find the maximum flow
        // through the path found.
        float path_flow = 10000000000.0;
        for (v = t; v != s; v = parent[v])
        {
            u = parent[v];
            path_flow = min(path_flow,  rGraph.getCapacityofAnEdge(u, v));
        }

        // update residual capacities of the edges and reverse edges
        // along the path
        for (v = t; v != s; v = parent[v])
        {
            u = parent[v];
            rGraph.setCapacityofAnEdge(u, v, rGraph.getCapacityofAnEdge(u, v) - (float)path_flow);
            rGraph.setCapacityofAnEdge(v, u, rGraph.getCapacityofAnEdge(v, u) + (float)path_flow);
        }
    }

    // Flow is maximum now, find vertices reachable from s
    bool visited[this->vertices];
    memset(visited, false, sizeof(visited));
    dfs(rGraph, s, visited);

    // Print all edges that are from a reachable vertex to
    // non-reachable vertex in the original graph
    for (int i = 0; i < this->vertices; i++)
        for (int j = 0; j < this->vertices; j++)
            if (visited[i] && !visited[j] && g.isAnEdgeFromTo(i, j)){
                cout << i << " - " << j << endl;
                g.deleteEdge(i, j);
            }
    return g;
}


float Graph::getCapacityofAnEdge(int u, int v) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found){
        if((u == get<0>(this->edges[i]) && v == get<1>(this->edges[i])) || (u == get<1>(this->edges[i]) && v == get<0>(this->edges[i])))
            found = true;
        else i++;
    }
    if(found)
        return get<2>(this->edges[i]);
    else
        return -1;
}

void Graph::setCapacityofAnEdge(int u, int v, float newValue) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found){
        if((u == get<0>(this->edges[i]) && v == get<1>(this->edges[i])) || (u == get<1>(this->edges[i]) && v == get<0>(this->edges[i])))
            found = true;
        else i++;
    }
    if(found)
        this->edges[i] = make_tuple(u, v, newValue);
}

bool Graph::isAnEdgeFromTo(int u, int v) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found){
        if(u == get<0>(this->edges[i]) && v == get<1>(this->edges[i]))
            found = true;
        else i++;
    }
    return found;
}

void Graph::deleteEdge(int u, int v) {
    int index = 0;
    bool found = false;
    while(index < this->edges.size() && !found){
        if(get<0>(this->edges[index]) == u && get<1>(this->edges[index]) == v)
            found = true;
        else
            index++;
    }
    if(found) {
        cout << "Deleting edge ... " << u << " -> " << v << endl;
        this->edges.erase(this->edges.begin() + index);
    }
}

// Cut the graph with the result of the algorithm
Graph Graph::doCut(Graph minCut){
    Graph cutedGraph(0);
    for(int i = 0; i < minCut.vertices - 1; i++){
        if(minCut.isAnEdgeFromTo(0, i)){
            cutedGraph.vertices++;
        }
    }
    for(int i = 2; i < cutedGraph.vertices; i++){
        if(minCut.isAnEdgeFromTo(i - 1, i))
            cutedGraph.addEdge(i - 1, i, minCut.getCapacityofAnEdge(i - 1, i));
        if(minCut.isAnEdgeFromTo(i, i - 1))
            cutedGraph.addEdge(i, i - 1, minCut.getCapacityofAnEdge(i, i - 1));
    }
    return cutedGraph;
}

// Retourner vecteur de vecteurs pour les graphes les plus denses
// Retourner vecteur de vecteurs pour toutes les composantes connexes
// Enlever les arêtes coupées par l'algo min cut