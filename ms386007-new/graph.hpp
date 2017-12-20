#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cstring>
#include <limits>

/**
 * Represents an undirected multigraph without loops using adjacency lists.
 */
template <typename TNode = unsigned int, typename TWeight = unsigned long>
class Graph {
public:
    enum build_t {MAX, SUM, MULTI};

    using adjacency_list = std::vector<std::pair<TWeight, TNode>>;
    using list_iter = typename adjacency_list::iterator;
    using node_list = std::vector<adjacency_list>;
    using size_type = typename node_list::size_type;

private:
    using hash_map = std::unordered_map<TNode, TNode>;
    using dehash_list = std::vector<TNode>;

    hash_map M;
    dehash_list N;
    node_list V;
    size_type nodeCount;

public:
    TNode hash(TNode v);
    TNode dehash(TNode v) { return N[v]; }

private:
    void addEdge(TNode v, TNode u, TWeight w);

public:
    Graph() : nodeCount(0) {}

    bool buildFromFile(std::string filename, build_t b);
    adjacency_list& getAdjacencyList(TNode v) { return V[hash(v)]; }
    size_type size() { return nodeCount; }
    list_iter sortAdjacencyList(TNode v,
                                list_iter begin,
                                list_iter end,
                                unsigned int k) {} // TODO: Implement sorting

};

template <typename TNode, typename TWeight>
TNode Graph<TNode, TWeight>::hash(TNode v) {
    if (M.find(v) == M.end()) {
        N.push_back(v);
        V.push_back(adjacency_list());
        return M[v] = nodeCount++;
    } else return M[v];
}

template <typename TNode, typename TWeight>
void Graph<TNode, TWeight>::addEdge(TNode v, TNode u, TWeight w) {
    if (v == u) return;
    TNode vHash = hash(v), uHash = hash(u);
    V[vHash].push_back(std::make_pair(w, u));
    V[uHash].push_back(std::make_pair(w, v));
}

template <typename TNode, typename TWeight>
bool Graph<TNode, TWeight>::buildFromFile(std::string filename, build_t b) {
    std::fstream fs;
    fs.open(filename, std::fstream::in);

    if (fs.fail()) {
        std::cerr << "Counld not open the file " << filename << "\n";
        fs.close();
        return false;
    }

    int c;
    while ((c = fs.peek()) == '#') {
        fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    TNode v, u;
    TWeight w;
    
    switch (b) {
    case MAX:
        while (!fs.eof()) {
            fs >> v >> u >> w;
            if (fs.fail()) break; // TODO: turn off failbit ?
            this->addEdge(v, u, w);
        }
        break;
    case SUM:
        while (!fs.eof()) {
            fs >> v >> u >> w;
            if (fs.fail()) break; // TODO: turn off failbit ?
            this->addEdge(v, u, w);
        }
        break;
    default:
        while (!fs.eof()) {
            fs >> v >> u >> w;
            if (fs.fail()) break; // TODO: turn off failbit ?
            this->addEdge(v, u, w);
        }
        break;
    }

    fs.close();
    return true;
}

#endif /* __GRAPH_HPP__ */