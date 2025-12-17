#include "uasisi/core/orchestrator.hpp"
#include <exception>
#include <iostream> 
#include <ostream>
#include <string>

int main(int argc, char** argv) {
    if(argc < 2){
        std::cerr << "Usage: " << argv[0] << "<PATHTOCONFIG>.yaml" << std::endl;
        return 1;
    }

    try{
        std::string config_file = argv[1];
        uasisi::Orchestrator orch;
        orch.load_config(config_file);
        return 0;
    } catch (const std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
