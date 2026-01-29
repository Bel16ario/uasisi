#ifndef SIGNALS_HPP
#define SIGNALS_HPP

#include "uasisi/core/types.hpp" 
#include <map>
#include <stdexcept>
#include <type_traits>

namespace uasisi{

class SignalMgr{
    public:

    SignalMgr() = default;
    ~SignalMgr() = default;

    template<typename T>
    void registerSignal(const std::string& name, const SpanwiseVec<T>& x0);

    template<typename T>
    const SpanwiseVec<T>& getReadOnly(const std::string& name) const;

    template<typename T>
    SpanwiseVec<T>& getWritable(const std::string& name);

    bool has(const std::string& name) const;

    void saveState(const std::string& filename); // Add later. should be called from the orchestrator. Orchestrator should have master save function that saves to same file states from signal manager + internal module states + orchestrator state (time, config, etc)
    void loadState(const std::string& filename); // Add later. Should read from unified checkpoint. 

    size_t size() const{return this->n;}

    private:
    std::map<std::string, SpanwiseVec<double>> double_signals;
    std::map<std::string, SpanwiseVec<std::vector<double>>> vector_signals;
    std::map<std::string, SpanwiseVec<airfoil>> airfoil_signals;
    size_t n = 0;
};

template<typename T>
void SignalMgr::registerSignal(const std::string & name, const SpanwiseVec<T>& x0){

    if(this->has(name)){
    
        throw std::runtime_error("Signal already exists");

    }

    if constexpr (std::is_same_v<T, double>) {
   
        this->double_signals[name] = x0;

    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
   
        this->vector_signals[name] = x0;

    } else if constexpr (std::is_same_v<T, airfoil>) {
   
        this->airfoil_signals[name] = x0;

    } else {
        
        throw std::runtime_error(std::string("Incompatible signal type: ") + typeid(T).name());

    }

    this->n++;

}

template<typename T>
const SpanwiseVec<T>& SignalMgr::getReadOnly(const std::string& name) const{
    
    if(this->has(name)){

        if constexpr (std::is_same_v<T, double>){
            
            return this->double_signals.at(name);

        } else if constexpr (std::is_same_v<T, std::vector<double>>){
            
            return this->vector_signals.at(name);

        } else if constexpr (std::is_same_v<T, airfoil>){
            
            return this->airfoil_signals.at(name);

        } else {
            
            throw std::runtime_error(std::string("Incompatible signal type: ") + typeid(T).name());

        }

    } else {
        throw std::runtime_error("Signal does not exist: " + name);
    }

}

template<typename T>
SpanwiseVec<T>& SignalMgr::getWritable(const std::string& name){
    
    if(this->has(name)){

        if constexpr (std::is_same_v<T, double>){
            
            return this->double_signals[name];

        } else if constexpr (std::is_same_v<T, std::vector<double>>){
            
            return this->vector_signals[name];

        } else if constexpr (std::is_same_v<T, airfoil>){
            
            return this->airfoil_signals[name];

        } else {
            
            throw std::runtime_error(std::string("Incompatible signal type: ") + typeid(T).name());

        }

    } else {
        throw std::runtime_error("Signal does not exist: " + name);
    }

}

bool SignalMgr::has(const std::string& name) const {
    return(
      this->double_signals.find(name) != this->double_signals.end() ||  
      this->vector_signals.find(name) != this->vector_signals.end() ||  
      this->airfoil_signals.find(name) != this->airfoil_signals.end() 
    );
}

}

#endif 
