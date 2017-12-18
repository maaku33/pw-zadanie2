#include "blimit.hpp"
#include <iostream>
#include <fstream>
#include <limits>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <queue>
#include <numeric>
#include <thread>
#include <mutex>
#include <atomic>

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

/**
 * Represents a graph using adjacency lists for all nodes. Hashes the nodes'
 * values upon edge creation, so that they span the interval [0, N) where
 * N is the number of nodes.
 */
class Graph {
public:
    using hash_map = std::unordered_map<node_t, node_t>;
    using adjacency_list = std::vector<std::pair<weight_t, node_t>>;
    using node_list = std::vector<adjacency_list>;
    using size_type = node_list::size_type;
    using dehash_list = std::vector<node_t>;

private:
    hash_map M;
    dehash_list N;
    node_list V;
    size_type nodeCount;

    node_t hashNode(node_t v);
    void resizeList(node_t v) { if (V.size() <= v) V.resize(v + 2); }

public:
    Graph() : nodeCount(0) {}

    void addEdge(node_t v, node_t u, weight_t w);
    node_list::const_iterator nodeItBegin() const { return V.begin(); }
    adjacency_list getAdjacencyList(node_t v) { return V[v]; }
    node_t dehashNode(node_t v) { return N[v]; }
    size_type size() { return nodeCount; }
};

node_t Graph::hashNode(node_t v) {
    if (M.find(v) == M.end()) {
        N.push_back(v);
        return M[v] = nodeCount++;
    } else return M[v];
}

void Graph::addEdge(node_t v, node_t u, weight_t w) {
    v = hashNode(v), u = hashNode(u);
    resizeList(std::max(v, u));

    V[v].push_back(std::make_pair(w, u));
    V[u].push_back(std::make_pair(w, v));
}

/**
 * Wraps different structures required in the parrallel execution of the
 * algorithm.
 */
class Matching {
public:
    using node_list = std::vector<node_t>;
    using atomic_node_list = std::vector<std::atomic<node_t>>;
    using count_list = std::vector<count_t>;
    using atomic_count_list = std::vector<std::atomic<count_t>>;

    node_list V;
    atomic_node_list Vdef;
    count_list B, S;
    atomic_count_list dB;

    Matching(Graph &G, count_t (*b)(count_t, node_t), count_t method);
};

Matching::Matching(Graph &G, count_t (*b)(count_t, node_t), count_t method) {
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);
    memset(&S, 0, G.size() * sizeof(decltype(S)::value_type));
    for (int i = 0; i < G.size(); i++) {
        B.push_back(b(method, G.dehashNode(i)));
    }
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

void parrallelExecutor(count_t start, count_t count,
                       Graph &G,
                       std::mutex &mut,
                       Matching &M) {
    node_t v, u;
    count_t i, j;
    Graph::adjacency_list A;
    for (count_t k = start; k < start + count && k < M.V.size(); k++) {
        v = M.V[k], i = 0, j = M.S[v];
        A = G.getAdjacencyList(v);

        while (i <= M.B[v] && j <= A.size()) {
            // let u be eligible partner of v
            mut.lock();
            // if u still eligible
                i++;
                // update u
            }
            mut.unlock();
    }
}

result_t parrallelBSuitor(Graph &G,
                          count_t (*b)(count_t, node_t),
                          count_t method,
                          count_t threads) {
    Matching M(G, b, method);
    std::mutex mut;

    while (!M.V.empty()) {
        count_t jump = M.V.size() / threads;
        std::vector<std::thread> T;

        for (count_t start = 0; start < M.V.size(); start += jump) {
            std::thread t([start, jump, &G, &mut, &M]{
                parrallelExecutor(start, jump, G, mut, M);
            });
            T.push_back(std::move(t));
        }
        while (!T.empty()) {
            T.back().join();
            T.pop_back();
        }

        M.V.insert(M.V.end(), M.Vdef.begin(), M.Vdef.end());
        M.Vdef.clear();
    }

    return b(method, (*(G.nodeItBegin()))[0].second);
}

int main(int argc, char** argv) {
    if (argc != 4) {
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
        std::cout << parrallelBSuitor(G, &bvalue, i, threadCount) << "\n";
    }

    return 0;
}
