#ifndef ORCHESTRATOR_HPP
#define ORCHESTRATOR_HPP


#include <map>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "uasisi/io/config.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/core/module.hpp"

namespace uasisi{

class Orchestrator {
public:
    
    Orchestrator();
    ~Orchestrator();

    void connectSystem();
    void init();
    void step(double t, double dt);

    void createModule(const std::string& moduleName, const std::string& instanceName, size_t prio);
    void configModules();
    //I have a dilemma, I need to configure each module. I am not sure but I think that since I have IModule pointers, I might need to dynamic cast to access the configuration methods in each module. I could return the pointer to the main file with the createModule() method but I don't want the main file to be able to access the init(), declare(), etc methods. I also want the configuration of each module to be done in the main file, ideally through helper functions. Maybe I could pass these config functions to the orchestrator as functionals or lambda functions or whatever they are called. They would need to be passed in the createModule method and stored in a map like the pointers and the priorities. Can you even have a map of functions?. All the config functions have the same return type (void) which maybe helps.

private:

    void connectSignal(const std::string& signalName, const std::vector<std::string>& moduleNames, const std::string& outputModule);
    void setExecOrder();

    std::map<std::string, IModule*> modules;
    std::map<std::string, size_t> priorities;

    std::vector<std::string> moduleOrder;

    std::map<std::string, std::unique_ptr<SpanwiseVec<double>>> signalsDOB;
    std::map<std::string, std::unique_ptr<SpanwiseVec<std::vector<double>>>> signalsVEC;
    std::map<std::string, std::unique_ptr<SpanwiseVec<airfoil>>> signalsAIR;
    std::map<std::string, std::unique_ptr<double>> signalsSCA;

    bool isConnected = false;
    bool signalsFiltered = false;
    bool isSet = false;
    bool isConfigured = false;
    bool execOrderIsSet = false;

};

}
#endif
