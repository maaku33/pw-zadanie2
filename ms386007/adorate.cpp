#include "blimit.hpp"
#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <vector>
#include <map>
#include <thread>

#ifdef DEBUG
    const bool debug = true;
#else
    const bool debug = false;
#endif

using count_t = unsigned int;
using node_t = unsigned long;
using weight_t = unsigned long;
using result_t = weight_t;

const count_t THREAD_MAX_N = 100;

class Graph {
public:
    using adjacency_list = std::vector<std::pair<weight_t, node_t>>;
    using node_list = std::map<node_t, adjacency_list>; // TODO: consider more efficient structures

private:
    node_list V;
    
public:
    Graph() = default;

    void addEdge(node_t v, node_t u, weight_t w);
    node_list::const_iterator nodeItBegin() const { return V.begin(); }
};

void Graph::addEdge(node_t v, node_t u, weight_t w) {
    // TODO: Merge same edges (leave heaviest) ?
    V[v].push_back(std::make_pair(w, u));
    V[u].push_back(std::make_pair(w, v));
}

bool createGraphFromFile(std::string &file, Graph &G) {
    std::fstream fs;
    fs.open(file, std::fstream::in);

    if (fs.fail()) {
        std::cerr << "Counld not open the file " << file << "\n";
        fs.close();
        return false;
    }

    int c;
    while ((c = fs.peek()) == '#') {
        fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    node_t v, u;
    weight_t w;
    while (!fs.eof()) {
        fs >> v >> u >> w;
        if (fs.fail()) break; // TODO: turn off failbit ?

        G.addEdge(v, u, w);
    }

    fs.close();
    return true;
}

result_t parrallelBSuitor(Graph &G,
                          count_t (*b)(count_t, node_t),
                          count_t method) {
    return b(method, ((*(G.nodeItBegin())).second)[0].second);
}

int main(int argc, char** argv) {
    if (argc != 4) { // invalid number of parameters
        std::cerr << "usage: " << argv[0] <<
            " thread-count inputfile b-limit\n"
            " thread-count\t\tnumber of created threads\n"
            " inputile\t\tpath to file containing the graph's description\n"
            " b-limit\t\tnumber of methods passed to bvalue function\n";
        
        return 1;
    }

    count_t threadCount = std::stoi(argv[1]), bLimit = std::stoi(argv[3]);
    std::string inputFilename(argv[2]);

    if (threadCount > THREAD_MAX_N) {
        std::cerr << "number of threads cannot exceed " << THREAD_MAX_N << "\n";

        return 1;
    }

    Graph G = Graph();
    if (!createGraphFromFile(inputFilename, G)) return 1;

    for (count_t i = 0; i < bLimit; i++) {
        std::cout << parrallelBSuitor(G, &bvalue, i) << "\n";
    }

    return 0;
}