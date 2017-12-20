#include "blimit.hpp"
#include <iostream>
#include <fstream>
#include <limits>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <cmath>
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

    hash_map M;

private:
    dehash_list N;
    node_list V;
    size_type nodeCount;

    node_t hashNode(node_t v);
    void resizeList(node_t v) { if (V.size() <= v) V.resize(v + 2); }

public:
    Graph() : nodeCount(0) {}

    void addEdge(node_t v, node_t u, weight_t w);
    node_list::const_iterator nodeItBegin() const { return V.begin(); }
    adjacency_list& getAdjacencyList(node_t v) { return V[v]; }
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

    V[v].push_back(std::make_pair(w, dehashNode(u)));
    V[u].push_back(std::make_pair(w, dehashNode(v)));
}

/**
 * Wraps different structures required in the parrallel execution of the
 * algorithm.
 */
class Matching {
    Graph &G;

public:
    using node_list = std::vector<node_t>;
    using count_list = std::vector<count_t>;
    using atomic_count_list = std::vector<std::atomic<count_t>>;
    using edge_set = std::set<std::pair<weight_t, node_t>, std::greater<std::pair<weight_t, node_t>>>;
    using set_list = std::vector<edge_set>;

    node_list V, Vdef;
    count_list B, T;
    atomic_count_list dB;
    set_list N, S;

    Matching(Graph &_G, count_t (*b)(count_t, node_t), count_t method);
    std::pair<weight_t, node_t> lastSuitor(node_t v);
    result_t result(); // TODO: result calculating
    node_t hashNode(node_t v) { return G.M[v]; }
    node_t dehashNode(node_t v) { return G.dehashNode(v); }
};

Matching::Matching(Graph &_G, count_t (*b)(count_t, node_t), count_t method)
    : G(_G) {
    T.resize(G.size());
    S.resize(G.size());
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);
    
    for (count_t i = 0; i < G.size(); i++) {
        B.push_back(b(method, G.dehashNode(i)));

        Graph::adjacency_list &tempA = G.getAdjacencyList(i);
        // if (B.back() > tempA.size()) 
            N.push_back(edge_set(tempA.begin(), tempA.end()));
        // else
            // N.push_back(edge_set(tempA.begin(), tempA.begin() + B.back()));
    }
}

result_t Matching::result() {
    result_t sum = 0;

    for (edge_set set : S) {
        for (std::pair<weight_t, node_t> p : set) {
            sum += p.first;
        }
    }
    return sum / 2;
}

std::pair<weight_t, node_t> Matching::lastSuitor(node_t v) {
    if (S[v].size() < B[v] || B[v] < 1) {
        return std::make_pair(0, -1); // TODO: node_t unsigned!
    }

    return std::make_pair((*(--(S[v].end()))).first, hashNode((*(--(S[v].end()))).second));
}

bool createGraphFromFile(std::string &file, Graph &G);
std::pair<weight_t, node_t> findEligiblePartner(node_t v, Matching &M);
bool isEligible(std::pair<weight_t, node_t> p, Matching &M);
void parrallelExecutor(count_t start, count_t count,
                       std::mutex &mut,
                       Matching &M);
result_t parrallelBSuitor(Graph &G,
                          count_t (*b)(count_t, node_t),
                          count_t method,
                          count_t threads);

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

    for (count_t i = 0; i <= bLimit; i++) {
        std::cout << parrallelBSuitor(G, &bvalue, i, threadCount) << "\n";
    }

    return 0;
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

std::pair<weight_t, node_t> findEligiblePartner(node_t v, Matching &M) {
    auto it = M.N[v].begin(), itEnd = M.N[v].end();

    while (it != itEnd && M.lastSuitor(M.hashNode((*it).second)).first >= (*it).first) ++it;

    if (it == itEnd) return std::make_pair(0, -1);
    return std::make_pair((*it).first, M.hashNode((*it).second));
}

bool isEligible(std::pair<weight_t, node_t> p, Matching &M) {
    return M.lastSuitor(p.second).first < p.first;
}

void parrallelExecutor(count_t start, count_t count,
                       std::mutex &mut,
                       Matching &M) {
    node_t v, u, y;
    std::pair<weight_t, node_t> p;

    for (count_t k = start; k < start + count && k < M.V.size(); k++) {
        v = M.V[k];

        while (M.T[v] < M.B[v] && !M.N[v].empty()) {
            mut.lock();
            p = findEligiblePartner(v, M);
            mut.unlock();

            if (p.second == (node_t) -1) break;

            mut.lock();

            if (isEligible(p, M)) {
                u = p.second;

                M.S[u].insert(std::make_pair(p.first, M.dehashNode(v)));
                M.N[v].erase(std::make_pair(p.first, M.dehashNode(p.second)));
                M.T[v]++;
                
                if (M.S[u].size() > M.B[u]) {
                    y = M.lastSuitor(u).second;
                    M.T[y]--;
                    M.Vdef.push_back(y);
                    M.S[u].erase(--(M.S[u].end()));
                }
            }

            mut.unlock();
        }
    }
}

result_t parrallelBSuitor(Graph &G,
                          count_t (*b)(count_t, node_t),
                          count_t method,
                          count_t threadCount) {
    Matching M(G, b, method);
    std::mutex mut;

    while (!M.V.empty()) {
        count_t jump = ceil((double)M.V.size() / threadCount);
        std::vector<std::thread> threads;

        for (count_t start = 0; start < M.V.size(); start += jump) {
            std::thread t([start, jump, &mut, &M]{
                parrallelExecutor(start, jump, mut, M);
            });

            threads.push_back(std::move(t));
        }

        // TODO: Use main thread

        while (!threads.empty()) {
            threads.back().join();
            threads.pop_back();
        }

        M.V = M.Vdef;
        M.Vdef.clear();
    }

    return M.result();
}
