#include "arbitraryOpt.hpp"
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <vector>

namespace uasisi{

ArbitraryOpt::ArbitraryOpt(){

    std::cout << "Aribitrary Optimizer created \n";

}

void ArbitraryOpt::init(){ // 
    
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->zIsSet || !this->lIsSet || !this->kIsSet || !this->zOutIsSet || !this->iTypeIsSet){
        throw std::runtime_error("Error, Aribitrary Optimizer Module not fully setup");
    }
    if(z.size() != l.size()){
        throw std::runtime_error("Z and L must have the same size.");
    }

    if(isConnected){ //Only if orchestrator actually connects output
        for(double& val : l){
            val *= k;
        }
        this->targetLift->set(this->zOut, uasisi::interpolate(this->z, this->l, this->zOut, getInterpType(this->iType)));
    }
    
    this->isSet = true;

}

std::vector<SignalInfo> ArbitraryOpt::declareSignals(){
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(1);
    reqSignals.push_back(SignalInfo("targetLift", DataType::DOB, SignalType::OUT));

    return reqSignals;
}

void ArbitraryOpt::validateConnections(){
    if(!this->zOutIsSet){ //The last check means that the orchestrator should size the SWVec before passing the pointer. Not sure if this is the best way since we need to set zOut here. Another option could be to pass an empty vector and reserve the space here. SWVec class already has checks for size of data and z. 
        throw std::runtime_error("Validation failed. Make sure zOut is set");
    }
    if(!targetLift){
        std::cout << "WARNING: output unconnected\n";
    } else {
        if(this->outputs.size() != 1 || this->outputs[0].name() != "targetLift" || this->targetLift->size() != this->zOut.size()){ //The last check means that the orchestrator should size the SWVec before passing the pointer. Not sure if this is the best way since we need to set zOut here. Another option could be to pass an empty vector and reserve the space here. SWVec class already has checks for size of data and z. 
            throw std::runtime_error("Validation failed");
        }
        this->isConnected = true;
    }
}

void ArbitraryOpt::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetLift"){
        this->targetLift = x;
        this->outputs.push_back(SignalInfo(name, DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }
}

void ArbitraryOpt::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void ArbitraryOpt::setZOut(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->zOut = zNew;
    this->zOutIsSet = true;
}

void ArbitraryOpt::setL(const std::vector<double>& lNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->l = lNew;
    this->lIsSet = true;
}

void ArbitraryOpt::setK(const double& kNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->k = kNew;
    this->kIsSet = true;
}

void ArbitraryOpt::setIType(const interpType& t){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = t;
    this->iTypeIsSet = true;
}

}

namespace{
static uasisi::ModuleRegistration<uasisi::ArbitraryOpt> registrationArbitraryOpt("arbitraryOpt");
}
