#include "blimit.hpp"
#include "graph.hpp"
#include <iostream>
#include <cstring>
#include <set>
#include <cmath>
#include <numeric>
#include <algorithm>
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
    std::mutex *M, *M2;
    std::mutex changeM;

public:
    struct Iter {
        using type = graph_t::adjacency_list::iterator;
        type last, sortEnd, end; 
        Iter(type l, type se, type e) : last(l), sortEnd(se), end(e) {}
    };

    using node_list = std::vector<node_t>;
    using count_list = std::vector<count_t>;
    using count_set = std::set<count_t>;
    using edge_set = std::set<edge_t, edge_comp>;
    using set_list = std::vector<edge_set>;
    using iter_list = std::vector<Iter>;

    node_list V;
    count_set Vdef;
    count_list B, T, dT;
    set_list S;
    iter_list N;

    Wrapper(graph_t &_G);

    node_t hash(node_t v) { return G.hash(v); }
    node_t dehash(node_t hv) { return G.dehash(hv); }
    void generateB(count_t (*b)(count_t, node_t), count_t method);
    edge_t lastSuitor(node_t v);
    edge_t bestCandidate(node_t hv);
    bool isSuitable(node_t hv, edge_t e);
    void addSuitor(node_t v, edge_t e);
    void lock(node_t v);
    void unlock(node_t v);
    result_t result();
    void reset();

    ~Wrapper() { delete [] M; delete [] M2; }
};

Wrapper::Wrapper(graph_t &_G) : G(_G) {
    T.resize(G.size());
    dT.resize(G.size());
    S.resize(G.size());
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);

    for (count_t i = 0; i < G.size(); i++) {
        graph_t::adjacency_list &A = G.getAdjacencyList(dehash(i));
        N.push_back(Iter(A.begin(), A.begin(), A.end()));
    }

    M = new std::mutex[G.size()];
    M2 = new std::mutex[G.size()];
}

void Wrapper::generateB(count_t (*b)(count_t, node_t), count_t method) {
    B.clear();
    for (count_t i = 0; i < G.size(); i++) {
        B.push_back(b(method, dehash(i)));
    }
}

edge_t Wrapper::lastSuitor(node_t v) {
    node_t hv = hash(v);
    
    std::unique_lock<std::mutex> lc(M2[hv]);

    if (B[hv] == 0 || S[hv].size() < B[hv]) return std::make_pair(0, -1);
    return *(--S[hv].end());
}

edge_t Wrapper::bestCandidate(node_t hv) {
    auto &it = N[hv].last, &sortEnd = N[hv].sortEnd;
    edge_t lastE;

    while (it != N[hv].end) {
        while (it != sortEnd && !(lastSuitor((*it).second) <
                                  std::make_pair((*it).first, dehash(hv)))) {
            ++it;
        }

        if (it == sortEnd) {
            sortEnd = G.sortAdjacencyList(sortEnd, N[hv].end, 9 * B[hv]);
        } else break;
    }

    if (it == N[hv].end) return std::make_pair(0, -1);
    return *it;
}

bool Wrapper::isSuitable(node_t hv, edge_t e) {
    return lastSuitor(e.second) < std::make_pair(e.first, dehash(hv));
}

void Wrapper::addSuitor(node_t v, edge_t e) {
    node_t hv = hash(v);

    std::unique_lock<std::mutex> lc(M2[hv]);

    S[hv].insert(e);
    if (S[hv].size() > B[hv]) {
        auto it = --S[hv].end();

        changeM.lock();
        Vdef.insert(hash((*it).second));
        dT[hash((*it).second)]--;
        changeM.unlock();

        S[hv].erase(it);
    }
}

void Wrapper::lock(node_t v) { M[hash(v)].lock(); }
void Wrapper::unlock(node_t v) { M[hash(v)].unlock(); }

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
    std::fill(T.begin(), T.end(), 0);
    S.clear();
    S.resize(G.size());
    V.resize(G.size());
    std::iota(V.begin(), V.end(), 0);

    for (count_t i = 0; i < G.size(); i++) {
        graph_t::adjacency_list &A = G.getAdjacencyList(dehash(i));
        N[i].last = A.begin();
    }
}

void parrallelExecutor(count_t start, count_t jump, Wrapper &W) {
    node_t v;
    edge_t e;

    for (count_t k = start; k < start + jump && k < W.V.size(); k++) {
        v = W.V[k];

        while (W.N[v].last != W.N[v].end && W.T[v] < W.B[v]) {            
            e = W.bestCandidate(v);

            if (e.second != (node_t) -1) {
                W.N[v].last++;
                
                W.lock(e.second);

                if (W.isSuitable(v, e)) {
                    W.T[v]++;
                    W.addSuitor(e.second, std::make_pair(e.first, W.dehash(v)));
                }

                W.unlock(e.second);
            } else {
                break;
            }
        }
    }
}

void parrallelBSuitor(graph_t &G, count_t threadCount, count_t bLimit) {
    Wrapper W(G);

    for (count_t i = 0; i <= bLimit; i++) {
        W.generateB(bvalue, i);

        while (!W.V.empty()) {
            count_t jump = ceil((double)W.V.size() / threadCount);
            std::vector<std::thread> threads;

            for (count_t start = jump; start < W.V.size(); start += jump) {
                std::thread t([start, jump, &W]{
                    parrallelExecutor(start, jump, W);
                });
                threads.push_back(std::move(t));
            }
            parrallelExecutor(0, jump, W);

            while (!threads.empty()) {
                threads.back().join();
                threads.pop_back();
            }

            W.V.clear();
            std::copy(W.Vdef.begin(), W.Vdef.end(), std::back_inserter(W.V));
            W.Vdef.clear();

            std::transform(W.dT.begin(), W.dT.end(), W.T.begin(), W.T.begin(),
                           std::plus<count_t>());
            std::fill(W.dT.begin(), W.dT.end(), 0);
        }
        
        std::cout << W.result() << std::endl;
        W.reset();
    }
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
    if (!G.buildFromFile(inputFilename)) return 1;

    parrallelBSuitor(G, threadCount, bLimit);

    return 0;
}