#include "blimit.hpp"
#include "graph.hpp"
#include <iostream>
#include <cstring>
#include <set>
#include <cmath>
#include <numeric>
#include <thread>
#include <mutex>

#ifdef DEBUG
    const bool debug = true;
#else
    const bool debug = false;
#endif

using count_t = unsigned int;
using node_t = unsigned long;
using weight_t = unsigned long;
using result_t = weight_t;
using graph_t = Graph<node_t, weight_t>;
using edge_t = std::pair<weight_t, node_t>;

const count_t THREAD_MAX_N = 100;

/**
 * Wraps different structures required in the execution of the algorithm.
 */
class Wrapper {
    using edge_comp = std::greater<edge_t>;

    graph_t &G;

    node_t hash(node_t v) { return G.hash(v); }
    node_t dehash(node_t v) { return G.dehash(v); }

public:
    struct Iter {
        using type = graph_t::adjacency_list::iterator;
        type last, sortEnd, end; 
        Iter(type l, type se, type e) : last(l), sortEnd(se), end(e) {}
    };

    using node_list = std::vector<node_t>;
    using count_list = std::vector<count_t>;
    using edge_set = std::set<edge_t, edge_comp>;
    using set_list = std::vector<edge_set>;
    using iter_list = std::vector<Iter>;

    node_list V, Vdef;
    count_list B;
    set_list S;
    iter_list N;

    Wrapper(graph_t &_G);

    void generateB(count_t (*b)(count_t, node_t), count_t method);
    edge_t lastSuitor(node_t v);
    edge_t bestCandidate(node_t v);
    bool isSuitable(edge_t e);
    result_t result();
    void reset();
};

Wrapper::Wrapper(graph_t &_G) : G(_G) {
    S.resize(G.size());
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);

    for (count_t i = 0; i < G.size(); i++) {
        graph_t::adjacency_list &A = G.getAdjacencyList(dehash(i));
        auto sortEnd = G.sortAdjacencyList(dehash(i), A.begin(), A.end(), 20); // TODO: Magic constant
        N.push_back(Iter(A.begin(), sortEnd, A.end()));
    }
}

void Wrapper::generateB(count_t (*b)(count_t, node_t), count_t method) {
    B.clear();
    for (count_t i = 0; i < G.size(); i++) {
        B.push_back(b(method, dehash(i)));
    }
}

edge_t Wrapper::lastSuitor(node_t v) {
    node_t hv = hash(v);
    if (B[hv] == 0 || S[hv].size() < B[hv]) return std::make_pair(0, -1);
    return *(--S[hv].end());
}

edge_t Wrapper::bestCandidate(node_t v) {
    node_t hv = hash(v);
    auto &it = N[hv].last, &sortEnd = N[hv].sortEnd;

    while (it != N[hv].end) {
        while (it != sortEnd && lastSuitor((*it).second).first >= (*it).first) {
            ++it;
        }

        if (it == sortEnd) {
            // TODO: sort next part of adjacency list
        } else break;
    }

    if (it == N[hv].end) return std::make_pair(0, -1);
    return *it;
}

bool Wrapper::isSuitable(edge_t e) {
    return lastSuitor(e.second).first < e.first;
}

result_t Wrapper::result() {
    result_t sum = 0;

    for (edge_set E : S) {
        for (edge_t e : E) {
            sum += e.first;
        }
    }

    return sum / 2;
}

void Wrapper::reset() {
    B.clear();
    S.clear();
    S.resize(G.size());
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);

    for (count_t i = 0; i < G.size(); i++) {
        graph_t::adjacency_list &A = G.getAdjacencyList(dehash(i));
        N[i].last = A.begin();
    }
}

void parrallelExecutor(count_t start, count_t jump, Wrapper &W, std::mutex *M) {
    node_t v, u, y;
    edge_t e;

    for (count_t k = start; k < start + jump && k < W.V.size(); k++) {
        v = W.V[k];

        // TODO: Implement the algorithm
    }
}

void parrallelBSuitor(graph_t &G, count_t threadCount, count_t bLimit) {
    Wrapper W = Wrapper(G);
    std::mutex *Mutex = nullptr;
    Mutex = new std::mutex[G.size()];

    for (count_t i = 0; i <= bLimit; i++) {
        W.generateB(bvalue, i);

        while (!W.V.empty()) {
            count_t jump = ceil((double)W.V.size() / threadCount);
            std::vector<std::thread> threads;

            for (count_t start = jump; start < W.V.size(); start += jump) {
                std::thread t([start, jump, &W, Mutex]{
                    parrallelExecutor(start, jump, W, Mutex);
                });
                threads.push_back(std::move(t));
            }

            parrallelExecutor(0, jump, W, Mutex);

            while (!threads.empty()) {
                threads.back().join();
                threads.pop_back();
            }

            W.V = W.Vdef;
            W.Vdef.clear();
        }
        
        W.reset();
    }

    delete [] Mutex;
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

    graph_t G = graph_t();
    if (!G.buildFromFile(inputFilename, graph_t::MAX)) return 1;

    parrallelBSuitor(G, threadCount, bLimit);

    return 0;
}