#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <limits>
#include <utility> 
#include <algorithm>
#include <string>
#include <cstdlib>
#include <map>

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph {

	public:
        
        /* define your data structure to represent a weighted undirected graph */
        map<string, map<string,T>> adj_list;            // Using adjacency list to represent the weighted undirected graph
                                                        // The map will store a list of values which are accessed using a key
        int edges = 0;                                  // Initiating the number of edges to be zero

        /* test1 */
		Graph(); // the contructor function.
		~Graph(); // the destructor function.
		size_t num_vertices(); // returns the total number of vertices in the graph.
		size_t num_edges(); // returns the total number of edges in the graph.

        /* test2 */
        void add_vertex(const string&); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
        bool contains(const string&); // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.
        
        /* test3 */
        vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

        /* test4 */
        void add_edge(const string&, const string&, const T&); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
        bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.
		
        /* test5 */
        vector<pair<string,string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.
        
        /* test6 */
        vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
        size_t degree(const string&); // returns the degree of a vertex.

        /* test7 */
		void remove_edge(const string&, const string&); // removes the edge between two vertices, if it exists.
        
        /* test8 */
        void remove_vertex(const string&); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

        /* test9 */
		vector<string> depth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.
		
        /* test10 */
        vector<string> breadth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.
        
        /* test11 */
		bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.
        
        /* test12 */
		Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.
		
};

/* test1  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
Graph<T>::Graph() {}

template <typename T>
Graph<T>::~Graph() {}


template <typename T>
size_t Graph<T>::num_vertices() {
    return adj_list.size();                             // Returns size of adjacency list as each row is defined by key of each vertex
}

template <typename T>
size_t Graph<T>::num_edges() {
    return edges;                                       // Returns the number of edges
}

/* test2  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
void Graph<T>::add_vertex(const string& u) {
    adj_list.insert(make_pair(u , map<string,T>()));    // Inserts key value 'u' into the adjacency list
                                                        // An inner map will be created with each vertex added which stores the adjacent vertices
}

template <typename T>
bool Graph<T>::contains(const string& u) {
    return (adj_list.count(u));                         // .count() will return 1 (true) if the element with key 'u' is found
}                                                       // and will return 0 (false) if not found
                                                        // Returns if the key 'u' can be found inside the graph

/* test3  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
vector<string> Graph<T>::get_vertices() {
    vector<string> vertices;                            // Creating a vector of strings named vertices

    for (auto i : adj_list) {                           // This loops for each vertex in the adjacency list as 'i' 
        vertices.push_back(i.first);                    // Since 'i' contains a key and value, add the key from 'i' into 'vertices'
    }                                                   // Basically for each vertex in the graph, add it to the vector 'vertices'

    return vertices;                                    // Return the vector 'vertices'
}

/* test4  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
void Graph<T>::add_edge(const string& u, const string& v, const T& weight) {
    edges++;                                            // Increment number of edges
    adj_list[u][v], adj_list[v][u] = weight;            // Set the weight of both [u][v] and [v][u] as the graph is undirected 
}

template <typename T>
bool Graph<T>::adjacent(const string& u, const string& v) {
    return (adj_list[u].count(v));                      // Basically, returns if 'v' can be found in the inner map of 'u' 
}                                                       // which would mean they are adjacent

/* test5  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
vector<pair<string,string>> Graph<T>::get_edges() {
    vector<string> vertices;                                            // This vector stores the vertices similar to test 3
    vector<pair<string,string>> edges;                                  // This vector stores the edges in string pairs ie. (A,B) would be an edge

    for (auto i : adj_list) {                                           // This loops for each main vertex in the graph
        for (auto j : i.second) {                                       // This loops for each adjacent vertex with respect to the first loop

            if (!count(vertices.begin(), vertices.end(), j.first )) {   // If the adjacent vertex cannot be found in vector 'vertices'
                edges.push_back(make_pair(i.first,j.first));            // Add the main and adjacent vertex into vector 'edges'
            }
        }
        vertices.push_back(i.first);                                    // Add vertex with key 'i' into the vector 'vertices' 
    }
    return edges;                                                       // Return the vector 'edges'

                                                                        // Basically, this method will go through each vertex in the graph
                                                                        // and within each vertex, add each adjacent vertex into a vector 'edges'
}                                                                       

/* test6  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string& u) {
    vector<string> neighbours;  
    for (auto i : adj_list[u]) {                                        // This loops for each adjacent vertex under the main vertex with key 'u'
        neighbours.push_back(i.first);                                  // Add neighbour vertex key 'i' into vector 'neighbours'
    }
    return neighbours;                                                  // Return the vector 'neighbours'

                                                                        // Basically, this adds each adjacent vertex found under vertex with key 'i' 
                                                                        // with the inputted key 'u' and returns them in a string vector
}

template <typename T>
size_t Graph<T>::degree(const string& u) {
    return get_neighbours(u).size();                                    // Returns the amount of neighbours that the main vertex with key 'u' has
}

/* test7  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
void Graph<T>::remove_edge(const string& u, const string& v) {
    if (adj_list.count(u) && adj_list.count(v)          // This tests if both keys 'u' and 'v' exist in the adjacency list
        && adjacent(u,v)) {                             // and also tests if they are adjacent to one another

            adj_list[u].erase(v);                       // Delete 'v' within 'u' in the adjacency list
            adj_list[v].erase(u);                       // Delete 'u' within 'v' in the adjacency list as well as the graph is undirected

             edges--;                                   // Decrement the number of edges
        }  
}                                                       // Basically, this checks if the vertices with keys 'u' and 'v' exist and are adjacent.
                                                        // If they are, then remove the adjacent vertex from the main vertex and vice versa since
                                                        // the map is undirected so (A,B) = (B,A). Finally, decrement the edges.

/* test8  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
void Graph<T>::remove_vertex(const string& u) {
    if (adj_list.count(u)) {                            // If the vertex with key 'u' exists in the adjacency list

        for (auto i : adj_list[u]) {                    // This loops for each adjacent vertex in the graph under key 'u'

            if (adjacent(u,i.first)) {                  // If key of 'i' and 'u' are adjacent
                adj_list[i.first].erase(u);             // Delete 'u' within 'i' in the adjacency list
                edges--;                                // Decrement edges
            }
        }
        adj_list.erase(u);                              // Erase the key 'u' from the adjacency list
    }
                                                        // Basically, this checks if a vertex with key 'u' exists in the graph and if so,
                                                        // it checks and deletes any edges connected to the vertex decrementing the number of 
                                                        // edges per deletion. When the vertex is disconnected, it erases it from map.
}

/* test9  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string& u) {
    vector<string> output;                                      // Creating a string vector to output the DFT of Graph
    stack<string> s;                                            // Creating a stack of string values
    s.push(u);                                                  // Add key 'u' into the stack to initiate
    
    while (!s.empty()) {                                        // Loop if the stack is not empty | Loop stops once all vertices are added to output
        string current = s.top();                               // Create a string to store the top value of the stack 
        s.pop();                                                // Delete the top of the stack as it has now been stored

        if (!count(output.begin(), output.end(), current)) {    // If current is not in the output vector yet
            output.push_back(current);                          // Add current to the output vector

            for (auto i : adj_list[current]) {                  // Looping over each adjacent vertex to the vertex at 'current' key 
                s.push(i.first);                                // Add the adjacent vertex into the stack
            }
        }
    }
    return output;                                              // Return the visiting order via DFS
}      
                                                                // Basically, a stack is used to store which elements are waiting to be stored.
                                                                // Starting from the vertex with key 'u', it adds the main vertex if not in 'output'.
                                                                // Then it moves to an adjacent vertex which become the main vertex. 
                                                                // This process repeats and if the vertex is not yet in 'output', it is added.
                                                                // In cases where the main vertex has no neighbour, it is deleted off the top
                                                                // of the stack and the previous vertex becomes the main vertex to check for any
                                                                // other adjacent vertices that have not been added yet.

/* test10  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string& u) {
    vector<string> output;                                      // The process is exactly the same as DFT except a queue is used instead of a stack
    queue<string> q;                                            // The difference is that in the previous test, a stack was used for removing elements,
    q.push(u);                                                  // the last element to be added is removed first, which allows for a DFT where the 
    while (!q.empty()) {                                        // last added vertex is removed so that it can return to a previous vertex.
        string current = q.front();
        q.pop();                                                // Here, using a queue means that whichever vertex was added first is removed first
        if (!count(output.begin(), output.end(), current)) {    // so the method will check ALL the adjacent vertices from the main vertex and add 
            output.push_back(current);                          // them before moving on to the next vertex. 
            for (auto i : adj_list[current]) {
                q.push(i.first);
            }
        }
    }
    return output; 
}

/* test11  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
bool Graph<T>::contain_cycles() {
    return (num_edges() >= num_vertices());                     // If there are more edges in the graph than vertices, 
                                                                // the graph will always contain a cycle
}

/* test12  ---------------------------------------------------------------------------------------------------------------------------------  */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree() {
    Graph<T> MST;                                                                   // Create a new graph to store MST            
    vector<string> visited;                                                         // A vector of strings to for all visited vertices

    visited.push_back(get_vertices().front());                                      // Add front vertex to the visited vector to start the MST
    MST.add_vertex(get_vertices().front());                                         // Add the same vertex to the MST graph

    while (MST.num_vertices() < num_vertices()) {                                   // If the MST has edges missing from the adj_list graph
        int minEdge = 9999;                                                         // Set a minimum weight to a large number which will be replaced
        pair<string,string> edge;                                                   // This pair of strings will be used to store the current edge
        for (auto i : visited) {                                                    // This loops for each vector 'visited'
            for (auto j : adj_list[i]) {                                            // This loops for each adjacent vertex inside of 'i' in visited

                if (!count(visited.begin(),visited.end(),j.first)                   // If the key 'j' is within 'i' of visited 
                 && j.second < minEdge ) {                                          // and the value of 'j' is smaller than the minimum weight

                    minEdge = j.second;                                             // Set the value of 'j' as the new minimum weight
                    edge = pair(i,j.first);                                         // Store the current edge into 'edge'
                }
            }
        }
        visited.push_back(edge.second);                                             // Add the adjacent vertex from the edge into the visited vertices
        MST.add_edge(edge.first, edge.second, adj_list[edge.first][edge.second]);   // Add the edge with the weight into the MST
    }
    return MST;                                                                     // Return the MST Graph
}

                                                                                    // Basically, we start by adding a vertex from adj_list to MST
                                                                                    // We add this vertex into the MST and into the 'visite' vertices
                                                                                    
                                                                                    // We check every edge between the visited vertex and each
                                                                                    // adjacent edge. We find the smallest edge and then add the two
                                                                                    // vertices into the MST with the weight of the edge between them.

                                                                                    // This is looped until all vertices from graph are added into MST. 
