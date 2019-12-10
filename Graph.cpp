//
// Created by Sandra on 23/10/2019.
//

#include "Graph.h"

// Constructor of a graph, initialization vertices and edges vectors
Graph::Graph() {
    this->vertices = vector<int>();
    this->edges = vector<tuple<int, int, float>>();
}

//------------- GETTERS & SETTERS -------------//

// Returns the index of a vertice stored in the vector's vertices. If it doesn't exist, returns -1
int Graph::getIndexOfAVertice(int vertice) {
    int index = 0;
    while(index < this->vertices.size())
        if(this->vertices[index] == vertice)
            return index;
        else
            index++;
    return -1;
}

// Returns the weight of an edge between two vertices. If it doesn't exist, returns -1
float Graph::getCapacityofAnEdge(int u, int v) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found)
        if((u == get<0>(this->edges[i]) && v == get<1>(this->edges[i])) || (u == get<1>(this->edges[i]) && v == get<0>(this->edges[i])))
            found = true;
        else i++;

    if(found)
        return get<2>(this->edges[i]);
    else
        return -1;
}

// Returns all the edges of a given vertice
vector<tuple<int, int, float>> Graph::getEdgesOfAVertice(int v){
    vector<tuple<int, int, float>> edgesOfV = vector<tuple<int, int, float>>();
    for(auto & edge : this->edges)
        if(get<0>(edge) == v || get<1>(edge) == v)
            edgesOfV.emplace_back(edge);
    return edgesOfV;
}

// Returns true if an edge from u to v exists. False if not
bool Graph::isAnEdgeFromTo(int u, int v) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found)
        if(u == get<0>(this->edges[i]) && v == get<1>(this->edges[i]) && get<2>(this->edges[i]) > 0)
            found = true;
        else i++;
    return found;
}

// Returns a list with all adjacent vertices of a given vertice
// Adjacent to a given vertice v means all vertices which are connected to v by an edge
list<int> Graph::getAdjacents(int v) {
    list<int> adjacents;

    for(auto & edge : this->edges)
        if(v == get<0>(edge))
            adjacents.push_back(get<1>(edge));
        else if(v == get<1>(edge))
            adjacents.push_back(get<0>(edge));

    adjacents.sort();
    adjacents.unique();
    return adjacents;
}

// Returns a graph built with a component
Graph Graph::getComponent(const bool *visited) {
    Graph result = Graph();
    for(int i = 1; i <= this->vertices.size(); i++)
        if(visited[i]) {
            result.vertices.emplace_back(i);
            result.addEdges(this->getEdgesOfAVertice(i));
        }
    return result;
}

// Returns a subgraph with only positives edges
Graph Graph::getPositiveGraph() {
    Graph positive = Graph();
    for(auto & edge : this->edges)
        if(get<2>(edge) >= 0) {
            positive.edges.emplace_back(edge);
            positive.addVertice(get<0>(edge));
            positive.addVertice(get<1>(edge));
        }
    return positive;
}

// Returns the total weight of a vertice : sum weights of all its edges
float Graph::getVerticeTotalWeight(int v) {
    float weight = 0;
    for(const auto & edge : this->getEdgesOfAVertice(v))
        weight += get<2>(edge);
    return weight;
}

// Returns the total weight of a graph
float Graph::getTotalWeight() {
    float weight = 0;
    for(auto & edge : this->edges)
        weight += get<2>(edge);
    return weight;
}

// Find the edge between two given vertices and change its value
void Graph::setCapacityofAnEdge(int u, int v, float newValue) {
    bool found = false;
    int i = 0;
    while(i < this->edges.size() && !found)
        if((u == get<0>(this->edges[i]) && v == get<1>(this->edges[i])))
            found = true;
        else i++;

    if(found) {
        this->edges[i] = make_tuple(u, v, newValue);
        cout << "OK SETTING " << u << "  " << v << "   " << newValue << endl;
        cout << "RESULTS IN " << get<0>(this->edges[i]) << "   " << get<1>(this->edges[i]) << "   " << get<2>(this->edges[i]) << endl;
    }
    else
        this->addEdge(u,v,newValue);
}

//------------- ADDING & DELETING -------------//

// Add a vertice to the graph
void Graph::addVertice(int vertice) {
    if(this->getIndexOfAVertice(vertice) == -1)
        this->vertices.emplace_back(vertice);
}

// Delete a vertice of the graph
void Graph::deleteVertice(int vertice) {
    int index = this->getIndexOfAVertice(vertice);
    if(index != -1)
        this->vertices.erase(this->vertices.begin() + index);
}

// Add an edge to the graph
void Graph::addEdge(int u, int v, float capacity) {
    this->edges.emplace_back(make_tuple(u, v, capacity));
}

// Add several edges to the graph
void Graph::addEdges(vector<tuple<int, int, float>> edgesToAdd){
    for(auto & edge : edgesToAdd)
        this->addEdge(get<0>(edge), get<1>(edge), get<2>(edge));
}

// Delete an edge of the graph
void Graph::deleteEdge(int u, int v) {
    int index = 0;
    bool found = false;
    while(index < this->edges.size() && !found)
        if (get<0>(this->edges[index]) == u && get<1>(this->edges[index]) == v)
            found = true;
        else
            index++;

    if(found) {
        cout << "Deleting edge ... " << u << " -> " << v << endl;
        this->edges.erase(this->edges.begin() + index);
    }
}

// Delete all edges of a vertice
void Graph::deleteEdgesOfAVertice(int vertice) {
    vector<tuple<int, int, float>> edgesToDelete = this->getEdgesOfAVertice(vertice);
    for(auto & edge : edgesToDelete)
        this->deleteEdge(get<0>(edge), get<1>(edge));
}

//------------- OPERATIONS ON GRAPH -------------//

// Depth-First Search to explore the graph by using adjacent vertices to v
void Graph::DFS(int v, bool *visited) {
    // Mark the current node as visited
    visited[v] = true;

    // Recur for all the vertices adjacent
    // to this vertex
    list<int> adj = this->getAdjacents(v);
    list<int>::iterator i;
    for (i = adj.begin(); i != adj.end(); ++i) {
        if (!visited[*i]) {
            DFS(*i, visited);
        }
    }
}

// Check if it's a connected graph with a dfs algorithm
// Fills the components' vector. 1 component means that the graph is connected, if more is not.
void Graph::checkConnection(int v, vector<bool *>*components) {
    // Mark all the vertices as not visited
    bool *visited = this->cleanVisited();

    // Call the recursive helper function
    DFS(v, visited);
    components->emplace_back(visited);
    for (int j = 1; j <= this->vertices.size(); j++)
        if (!visited[j]) {
            //his->checkConnection(j, components);
            visited = this->cleanVisited();
            DFS(j, visited);
            components->emplace_back(visited);
        }
    delete(visited);
}

// Init then returns a bool* where each value is false
bool* Graph::cleanVisited(){
    bool *visited = new bool[this->vertices.size()]; // Delete after new
    for (int i = 1; i <= this->vertices.size(); i++)
        visited[i] = false;
    return visited;
}

// Recursive function to get all clusters of a graph
// Get only positive edges of the graph
// Check if it's a connected graph
// If not : Run the function on each component
// If yes : Find a densest subgraph, store it then remove processed data of the graph and run the function on the new graph
// If there are vertices but no more edges, add vertices as new cluster
vector<vector<int>> Graph::greedyCC() {
    vector<vector<int>> result = vector<vector<int>>();

    // Get the positive graph of g
    Graph positive = this->getPositiveGraph();
    vector<bool *> components = vector<bool *>();

    // Check if the positive graph is connected
    positive.checkConnection(positive.vertices[0], &components);

    // Is not connected then proceed for each component
    if(components.size() != 1){
        vector<vector<int>> interResult = vector<vector<int>>();
        for(auto & component : components){
            interResult = positive.getComponent(component).greedyCC();
            for(auto & inter : interResult)
                result.emplace_back(inter);
        }
    }
    // Is connected so proceed while there are edges in the positive graph
    else{
        vector<int> X = positive.findDensestSubgraph(0.5);
        result.emplace_back(X);
        // Removing found cluster of the original graph
        positive = positive.removeVertices(X);
        if (!positive.edges.empty())
            positive.greedyCC();
    }
    // There are vertices but no more edges so add remaining vertices as new cluster
    if(positive.edges.empty() && !positive.vertices.empty())
        result.emplace_back(positive.vertices);
    return result;
}

vector<int> Graph::findDensestSubgraph(float delta) {
    // Initialization
    float weight = getTotalWeight();
    float min = 0, max = weight, minG = 0;
    bool done = false;
    vector<int> bestX = vector<int>();
    Graph cut = Graph();
    cout << "Start with min = " << min << " & max = " << max << endl;

    while(!done){
        minG = min + (max - min) / 2;
        cout << "MinG = " << minG << endl;
        cut = getMinCut(this->computeHfromG(weight, minG), 0, this->vertices.size() + 1);

        if(cut.getTotalWeight() >= (float)this->vertices.size() * weight)
            max = minG;
        else{
            cout << "We found a bestX" << endl;
            bestX = doCut(cut);
            min = minG;
        }
        if(max - min < delta)
            done = true;
    }
    if(bestX.empty()){
        bestX = this->vertices;
    }
    return bestX;
}

//------------- OPERATIONS ON DIRECTED GRAPH -------------//

// Returns h, a directed graph from g
// Extra vertice 0 : source
// Extra vertice this->vertices.size() + 1 : sink
Graph Graph::computeHfromG(float weight, float minG) {
    Graph h = Graph();
    h.addVertice(0);
    for(auto & vertice : this->vertices)
        h.addVertice(vertice);
    h.addVertice(this->vertices.size() + 1);

    for(auto & edge : this->edges){
        h.addEdge(0, get<0>(edge), weight);
        h.addEdge(get<0>(edge), get<1>(edge), get<2>(edge));
        h.addEdge(get<1>(edge), get<0>(edge), get<2>(edge));
        h.addEdge(get<0>(edge), h.vertices.size() - 1, weight + 2 * minG  - getVerticeTotalWeight(get<0>(edge)));
    }
    h.addEdge(0, h.vertices.size() - 2, weight);
    h.addEdge(h.vertices.size() - 2, h.vertices.size() - 1, weight + 2 * minG - getVerticeTotalWeight(h.vertices.size() - 2));
    return h;
}

// Returns True if there is a path from source 's' to sink 't' in residual graph
// Also fills parent[] to store the path
bool Graph::bfs(Graph rGraph, int s, int t, int parent[])
{
    // Create a visited array and mark all vertices as not visited
    bool visited[this->vertices.size()];
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
        for (int v = 0; v < this->vertices.size(); v++)
            if (!visited[v] && rGraph.isAnEdgeFromTo(u, v))
            {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
    }

    // If we reached sink in BFS starting from source, then return
    // true, else false
    return visited[t];
}

// Returns all vertices reacheable from source
void Graph::dfs(Graph rGraph, int s, bool visited[]) {
    visited[s] = true;
    for (int i = 0; i < this->vertices.size(); i++)
        if (rGraph.isAnEdgeFromTo(s, i) && !visited[i])
            dfs(rGraph, i, visited);
}

// Returns a cut graph by a bfs then a dfs
Graph Graph::getMinCut(Graph g, int s, int t)
{
    int u, v;
    // Create a residual graph and fill the residual graph with
    // given capacities in the original graph as residual capacities
    // in residual graph
    Graph rGraph = g;
    Graph cut = Graph();

    int parent[this->vertices.size()];  // This array is filled by BFS and to store path

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
    bool visited[this->vertices.size() + 2];
    memset(visited, false, sizeof(visited));
    dfs(rGraph, s, visited);

    // Store all edges that are from a reachable vertex to
    // non-reachable vertex in the original graph
    for (int i = 0; i <= g.vertices.size() + 1; i++)
        for (int j = 0; j <= g.vertices.size() + 1; j++)
            if (visited[i] && !visited[j] && g.isAnEdgeFromTo(i, j)){
                cut.addVertice(i);
                cut.addVertice(j);
                cut.addEdge(i, j, g.getCapacityofAnEdge(i, j));
            }
    return cut;
}

// Cut the graph with the result of the algorithm
vector<int> Graph::doCut(Graph minCut){
    vector<int> cutGraph = vector<int>() ;
    for(int i = 1; i < minCut.vertices.size() - 1; i++)
        if(!minCut.isAnEdgeFromTo(0, i))
            cutGraph.emplace_back(i);
    return cutGraph;
}

// Returns a graph without the vertices to remove after min cut algorithm
Graph Graph::removeVertices(const vector<int>& verticesToRemove) {
    Graph result = *this;
    for(int vertice : verticesToRemove) {
        result.deleteEdgesOfAVertice(vertice);
        result.deleteVertice(vertice);
    }
    return result;
}

//------------- DISPLAYING -------------//

// Display all adjacent vertices of a given vertice
void Graph::printAdjacents(int vertice) {
    list<int> voisins = this->getAdjacents(vertice);
    list<int>::iterator i;
    for (i = voisins.begin(); i != voisins.end(); ++i)
        printf("Neighbor %d : %d \n", vertice, *i);
}

// Display all edges of a graph
void Graph::printEdges() {
    for(auto & edge : this->edges)
        cout << "Edge from " << get<0>(edge) << " to " << get<1>(edge) << " with a capacity of " << get<2>(edge) << endl;
}

// Display all vertices of a graph
void Graph::printVertices() {
    for(auto & vertice : this->vertices)
        cout << vertice << " ";
    cout << endl;
}

// Display all components of a graph
void Graph::printComponents(const vector<bool *>& components){
    for(auto & component : components){
        cout << "component : " << component << endl;
        for(int i = 1; i <= this->vertices.size(); i++)
            cout << component[i] << endl;
    }
}

// Display a cluster
void Graph::printCluster(vector<int> &clusterToPrint) {
    cout << "Cluster : ";
    for(int vertice : clusterToPrint)
        cout << vertice << " ";
    cout << endl;
}

// Display several clusters
void Graph::printClusters(vector<vector<int>> &clustersToPrint) {
    for(auto & i : clustersToPrint)
        printCluster(i);
}

//------------- GENERATING DATA -------------//

// Generate a random graph to test algorithm
Graph Graph::generateRandomGraph(int nbVertices) {
    Graph result = Graph();
    for(int i = 1; i <= nbVertices; i++)
        result.addVertice(i);
    int max = 10, min = -10;
    for(int i = 1; i <= nbVertices; i++)
        for(int j = i; j <= nbVertices; j++)
            if(i != j)
                result.addEdge(i, j, float (rand()%(max - min + 1) + min) / 10);
    return result;
}

// Generate a graph with real data
Graph Graph::generateGraph(vector<tuple<int, int, float>> data) {
    Graph result = Graph();
    for(auto & line : data){
        result.addVertice(get<0>(line));
        result.addVertice(get<1>(line));
        result.edges.emplace_back(line);
    }
    return result;
}


