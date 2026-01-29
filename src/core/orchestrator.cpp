#include "uasisi/core/orchestrator.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

namespace uasisi {
    
Orchestrator::Orchestrator(){
    std::cout << "Orchestrator created" << std::endl;
}

Orchestrator::~Orchestrator(){
    std::cout << "Orchestrator terminated" << std::endl;
}

void Orchestrator::connectSystem(){ //Not sure if this is the fastest, most efficient way but
    // 1 - Find which signals are requested by multiple modules.
    // 2 - From these signals, verify that they have a minimum of one input and one output, and a maximum of one output
    // 3 - Look again to find all the modules that require each valid signal and connect them
    if(this->isConnected || this->isSet || this->signalsFiltered){
        throw std::runtime_error("Orchestrator already connected/set");
    }
    if(this->modules.size() != this->priorities.size() || this->modules.empty()){
        throw std::runtime_error("Empty or mismatched module list");
    }

    std::map<std::string, std::vector<std::string>> allSignals;
    std::map<std::string, size_t> nInputSignals;
    std::map<std::string, size_t> nOutputSignals;
    std::map<std::string, std::string> outputModules; // This will require a rework if this ever supports buffers

    std::string compoundName;

    for(const auto& [instanceName, module] : this->modules){
        for(const SignalInfo& signal : module->declareSignals()){
            compoundName = signal.name();
            switch(signal.dataType()){
                case DataType::DOB:
                    compoundName += "DOB";
                    break;
                case DataType::VEC:
                    compoundName += "VEC";
                    break;
                case DataType::AIR:
                    compoundName += "AIR";
                    break;
                case DataType::SCA:
                    compoundName += "SCA";
                    break;
                default:
                    throw std::runtime_error("Unknown data type for signal: " + compoundName);
                    break;
            }
            auto it = allSignals.find(compoundName);
            if(it == allSignals.end()){
                std::vector<std::string> signalModules;
                signalModules.push_back(instanceName);
                allSignals[compoundName] = signalModules;
                switch(signal.signalType()){ //Buffer not implemented yet
                    case SignalType::IN:
                        nInputSignals[compoundName] = 1;
                        nOutputSignals[compoundName] = 0;
                        break;
                    case SignalType::OUT:
                        nInputSignals[compoundName] = 0;
                        nOutputSignals[compoundName] = 1;
                        outputModules[compoundName] = instanceName;
                        break;
                    default:
                        throw std::runtime_error("Unknown or not supported signal type for signal: " + compoundName);
                        break;
                }
            } else {
               it->second.push_back(instanceName); 
                switch(signal.signalType()){ //Buffer not implemented yet
                    case SignalType::IN:
                        nInputSignals[compoundName]++;;
                        break;
                    case SignalType::OUT:
                        nOutputSignals[compoundName]++;
                        outputModules[compoundName] = instanceName; //Overwrites since only one output allowed.
                        break;
                    default:
                        throw std::runtime_error("Unknown or not supported signal type for signal: " + compoundName);
                        break;
                }
            }
        }
    }
    for(auto it = allSignals.begin(); it != allSignals.end(); ){
        if(nInputSignals[it->first] < 1 || nOutputSignals[it->first] != 1){
            std::cout << "Abandoning hanging or invalid signal: " << it->first << "\n"; //I should probably print wether the signal was hanging or invalid (more than one output) but im lazy.
            it = allSignals.erase(it);
        } else {
            it++;
        }
    }
    this->signalsFiltered = true;
    for(const auto& [signal, modulesList] : allSignals){
        this->connectSignal(signal, modulesList, outputModules[signal]);
    }
    this->isConnected = true;
}

void Orchestrator::init(){ //I want to move the timestepping to the main file.
    if(this->isSet || !this->isConnected || !this->isConfigured){
        throw std::runtime_error("Orchestrator not set properly or already running");
    }
    this->setExecOrder();
    for(const std::string& module : this->moduleOrder){
        this->modules[module]->init();
    }
    this->isSet = true;
}

void Orchestrator::step(double t, double dt){
    if(!this->isSet || !this->isConnected || !this->isConfigured || !this->execOrderIsSet){
        throw std::runtime_error("Module not initialized");
    }
    for(const std::string& module : this->moduleOrder){
        this->modules[module]->step(t, dt);
    }

}

void Orchestrator::createModule(const std::string& moduleName, const std::string& instanceName, size_t prio){
    if(this->isSet || this->isConnected || this->execOrderIsSet){
        throw std::runtime_error("Orchestrator already running, connected or module order has been frozen");
    }
    if(this->modules.find(instanceName) != this->modules.end()){
        throw std::runtime_error("A module is already instantiated with that name");
    }
    for(const auto& [_ , p] : this->priorities){
        if(p == prio){
            std::runtime_error("A module with that priority already exists");
        }
    }
    this->modules[instanceName] = ModuleFactory<IModule>::instance().create(moduleName); //Maybe later I can add module type to the registration information to allow the orchestrator to implement a method that returns all the modules of a specific type and thus in the main file, logic can be used to always pick one of a certain type. I do want to keep this architecture where the modules are created from IModule to allow for new module types without necessarily changing a lot of the core code. A module can even inherit directly from IModule.
    this->priorities[instanceName] = prio;
}

void Orchestrator::connectSignal(const std::string signal&, const std::vector<std::string>& moduleNames, const std::string& outputModule){
    if(this->isConnected || this->isSet || !this->signalsFiltered){
        throw std::runtime_error("Orchestrator already connected/set");
    }
    std::string dType = signal.substr(signal.length() - 3);
    std::string signalName = signal.substr(0, signal.length() - 3);
    if(dType == "DOB"){
        if(this->signalsDOB.find(signalName) != this->signalsDOB.end()){
            throw std::runtime_error("Signal already exists");
        }
        std::unique_ptr<SpanwiseVec<double>> ptrDOB = std::make_unique<SpanwiseVec<double>>();
        SpanwiseVec<double>* rawPtrDOB = ptrDOB.get();
        this->signalsDOB[signalName] = std::move(ptrDOB);
        std::cout << "Double valued signal " << signalName << " created\n";
        for(const std::string name : moduleNames){
            if(this->modules.find(name) == this->modules.end()){
                throw std::runtime_error("Module " + name + " does not exist");
            }
            if(name == outputModule){
                this->modules[name]->connectOutputDouble(signalName, rawPtrDOB);
            } else {
                this->modules[name]->connectInputDouble(signalName, rawPtrDOB);
            }
            std::cout << signalName << " connected to " + name;
        }
    } else if(dType == "VEC"){
        if(this->signalsVEC.find(signalName) != this->signalsVEC.end()){
            throw std::runtime_error("Signal already exists");
        }
        std::unique_ptr<SpanwiseVec<std::vector<double>>> ptrVEC = std::make_unique<SpanwiseVec<std::vector<double>>>();
        SpanwiseVec<std::vector<double>>* rawPtrVEC = ptrVEC.get();
        this->signalsVEC[signalName] = std::move(ptrVEC);
        std::cout << "Vector valued signal " << signalName << " created\n";
        for(const std::string name : moduleNames){
            if(this->modules.find(name) == this->modules.end()){
                throw std::runtime_error("Module " + name + " does not exist");
            }
            if(name == outputModule){
                this->modules[name]->connectOutputVector(signalName, rawPtrVEC);
            } else {
                this->modules[name]->connectInputVector(signalName, rawPtrVEC);
            }
            std::cout << signalName << " connected to " + name;
        }
    } else if(dType == "AIR"){
        if(this->signalsAIR.find(signalName) != this->signalsAIR.end()){
            throw std::runtime_error("Signal already exists");
        }
        std::unique_ptr<SpanwiseVec<airfoil>> ptrAIR = std::make_unique<SpanwiseVec<airfoil>>();
        SpanwiseVec<airfoil>* rawPtrAIR = ptrAIR.get();
        this->signalsAIR[signalName] = std::move(ptrAIR);
        std::cout << "airfoil valued signal " << signalName << " created\n";
        for(const std::string name : moduleNames){
            if(this->modules.find(name) == this->modules.end()){
                throw std::runtime_error("Module " + name + " does not exist");
            }
            if(name == outputModule){
                this->modules[name]->connectOutputAirfoil(signalName, rawPtrAIR);
            } else {
                this->modules[name]->connectInputAirfoil(signalName, rawPtrAIR);
            }
            std::cout << signalName << " connected to " + name;
        }
    } else if(dType == "SCA"){
        if(this->signalsSCA.find(signalName) != this->signalsSCA.end()){
            throw std::runtime_error("Signal already exists");
        }
        std::unique_ptr<double> ptrSCA = std::make_unique<double>();
        double* rawPtrSCA = ptrSCA.get();
        this->signalsSCA[signalName] = std::move(ptrSCA);
        std::cout << "Scalar valued signal " << signalName << " created\n";
        for(const std::string name : moduleNames){
            if(this->modules.find(name) == this->modules.end()){
                throw std::runtime_error("Module " + name + " does not exist");
            }
            if(name == outputModule){
                this->modules[name]->connectOutputScalar(signalName, rawPtrSCA);
            } else {
                this->modules[name]->connectInputScalar(signalName, rawPtrSCA);
            }
            std::cout << signalName << " connected to " + name;
        }
    } else {
        throw std::runtime_error("Unknown data type for signal: " + signalName);
    }

}

void Orchestrator::setExecOrder(){
    if(this->isSet || !this->isConnected){
        throw std::runtime_error("Orchestrator already running or not connected yet"); //To prevent additional modules from being added after this has been executed.
    }
    std::vector<size_t> prios;
    std::vector<size_t> idx;
    std::vector<std::string> names;
    prios.resize(this->priorities.size());
    idx.resize(this->priorities.size());
    this->moduleOrder.reserve(this->priorities.size());

    size_t i = 0;
    for(const auto& [name, p] : this->priorities){
        idx.push_back(i);
        prios.push_back(p);
        names.push_back(name);
        i++;
    }

    std::sort(idx.begin(), idx.end(), [&prios](size_t p0, size_t p1){return prios[p0] < prios[p1];});
    for(size_t i : idx){
        this->moduleOrder.push_back(names[i]);
    }

    this->execOrderIsSet = true;
}

}
