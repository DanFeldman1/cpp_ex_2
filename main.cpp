#include "my_algorithm.h"
#include "my_simulator.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    MySimulator simulator;

    // Check if the correct number of arguments are provided
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <houseFilePath>" << std::endl;
        return 1;
    }

    // The file path is expected to be the first argument
    std::string houseFilePath = argv[1];

    simulator.readHouseFile(houseFilePath);
    MyAlgorithm algo;
    simulator.setAlgorithm(algo);
    simulator.run();



    // Extract the output file path
    std::string houseFileName = houseFilePath;
    size_t lastSlashPos = houseFilePath.find_last_of("/\\");
    if (lastSlashPos != std::string::npos) {
        houseFileName = houseFilePath.substr(lastSlashPos + 1);
    }
    std::string outputFileName = "output_" + houseFileName;

    // Write the output file
    simulator.writeOutputFile(outputFileName);

    
    return 0;
}
