#include <iostream>
#include <string>
#include <thread>

#ifdef DEBUG
    const bool debug = true;
#else
    const bool debug = false;
#endif

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: " << argv[0] <<
            " thread-count inputfile b-limit\n"
            " thread-count\t\tnumber of created threads\n"
            " inputile\t\tpath to file containing the graph's description\n"
            " b-limit\t\tmaximum number of edges sharing a single vertex\n";
        
        return 1;
    }

    int threadCount = std::stoi(argv[1]), bLimit = std::stoi(argv[3]);
    std::string inputFilename(argv[2]);


}