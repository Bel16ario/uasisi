#ifndef ORCHESTRATOR_HPP
#define ORCHESTRATOR_HPP


#include <functional>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

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

    void createModule(const std::string& moduleName, const std::string& instanceName, size_t prio, std::function<void(IModule*)> configurator);
    void configureModules();
    //I have a dilemma, I need to configure each module. I am not sure but I think that since I have IModule pointers, I might need to dynamic cast to access the configuration methods in each module. I could return the pointer to the main file with the createModule() method but I don't want the main file to be able to access the init(), declare(), etc methods. I also want the configuration of each module to be done in the main file, ideally through helper functions. Maybe I could pass these config functions to the orchestrator as functionals or lambda functions or whatever they are called. They would need to be passed in the createModule method and stored in a map like the pointers and the priorities. Can you even have a map of functions?. All the config functions have the same return type (void) which maybe helps.

    IModule* getModule(const std::string& instanceName); // I am not sure how safe this is, returning a raw pointer, but I like this solution of only breaking abstraction on user request.
    const SpanwiseVec<double>* getDOB(const std::string& signalName);
    const SpanwiseVec<std::vector<double>>* getVEC(const std::string& signalName);
    const SpanwiseVec<airfoil>* getAIR(const std::string& signalName);
    const double* getSCA(const std::string& signalName);

private:

    void connectSignal(const std::string& signalName, const std::vector<std::string>& moduleNames, const std::string& outputModule);
    void setExecOrder();

    std::map<std::string, std::unique_ptr<IModule>> modules;
    std::map<std::string, size_t> priorities;
    std::map<std::string, std::function<void(IModule*)>> configurators;

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
