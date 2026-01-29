#ifndef MODULE_HPP
#define MODULE_HPP

#include "uasisi/io/config.hpp"
#include "uasisi/core/types.hpp"
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace uasisi{

template<typename T>
class ModuleFactory { //My best attempt at a singleton + factory

    public:


    static ModuleFactory& call();

    void registerModule(const std::string& name, std::function<std::unique_ptr<T>()> creator);
    std::unique_ptr<T> create(const std::string& name) const;
    std::vector<std::string> getRegisteredModules() const;
    bool isRegistered(const std::string& name) const;

    private:
    
    ModuleFactory() = default;
    ~ModuleFactory() = default;

    std::map<std::string, std::function<std::unique_ptr<T>()>> creators;

};

enum class DataType{
  
    DOB, //Double
    VEC, //Vector of doubles
    AIR, //Airfoils
    SCA, //Scalar

};

enum class SignalType{
  
    IN, //Input
    OUT, //Output
    BUF, //Buffer

};

class SignalInfo{

    public:
    
    SignalInfo(const std::string& name, const DataType& dT, const SignalType&  sT){this->name_ = name; this->dType = dT; this->sType = sT;};
    ~SignalInfo() = default;

    std::string name(){return this->name_;}
    DataType dataType(){return this->dType;}
    SignalType signalType(){return this->sType;}


    private:

    std::string name_;
    DataType dType;
    SignalType sType;

};

class IModule {

    public:

    IModule();
    virtual ~IModule() = default;

    virtual void init(const Config& config) = 0; // Do I really need to pass the config here? Is this the best way to do this?

    virtual std::vector<SignalInfo> declareSignals() = 0;
    virtual void validateConnections() = 0;

    virtual void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){}
    virtual void connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x){}
    virtual void connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x){}
    virtual void connectInputScalar(const std::string& name, const double* x){}
    virtual void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){}
    virtual void connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x){}
    virtual void connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x){}
    virtual void connectOutputScalar(const std::string& name, double* x){}

    virtual void step(double t, double dt) = 0; //returns when processing is done.
    
    virtual const std::string& getName() const = 0;
    //virtual const Config& config() const = 0;

    protected:

    std::vector<SignalInfo> inputs;
    std::vector<SignalInfo> outputs;
    std::vector<SignalInfo> buffers;


};

class IOptimizer : public IModule{
    
    public:

    private:

};

class IControl : public IModule{

    public:

    private:

};

class IActuator : public IModule{

    public:

    private:

};

class IPhysics : public IModule{

    public:

    private:

};

class ISensor : public IModule{

    public:

    private:

};

class IMonitor : public IModule{

    public:

    private:

};

template<typename T>
ModuleFactory<T>& ModuleFactory<T>::call(){
    static ModuleFactory factory;
    return factory;
}

template<typename T>
void ModuleFactory<T>::registerModule(const std::string& name, std::function<std::unique_ptr<T>()> creator){
    if(this->creators.find(name) != this->creators.end()){
        throw std::runtime_error("Module already registered under the same name:" +  name);
    }
    creators[name] = creator;
    std::cout << "Registered " << typeid(T).name() << ": " << name << "\n";
}

template<typename T>
std::unique_ptr<T> ModuleFactory<T>::create(const std::string& name) const {
    auto it = this->creators.find(name);
    if(it == this->creators.end()){
        throw std::runtime_error("Unknown Module for " + typeid(T).name() + ": " + name);
    }
    return it->second();
}

template<typename T>
std::vector<std::string> ModuleFactory<T>::getRegisteredModules() const {
    std::vector<std::string> names;
    for(const auto& [name, _] : this->creators){
        names.push_back(name);
    }
    return names;
}

template<typename T>
bool ModuleFactory<T>::isRegistered(const std::string& name) const {
    return this->creators.find(name) != this->creators.end();
}

template<typename U>
class ModuleRegistration {
    public:
    explicit ModuleRegistration(const std::string& name){
        ModuleFactory<uasisi::IModule>::call().registerModule(name, []() {return std::make_unique<U>();});
    }
};

}

#endif
