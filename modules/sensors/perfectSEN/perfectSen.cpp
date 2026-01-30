#include "perfectSen.hpp"
#include <iostream>
#include <stdexcept>

namespace uasisi {

PerfectSen::PerfectSen(){
 std::cout << "Perfect Sensor created\n";
}

void PerfectSen::init(const Config& config){
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->dTypeIsSet || !this->iTypeIsSet){
        throw std::runtime_error("Error, Perfect Actuator Module not fully setup");
    }
    if(this->isConnected){
        if(!this->sLiftConnected){
            //nothing
        } else if(!this->rLiftConnected){
            //For this perfect sensor, a vector of zeros will be passed when input is unconnected
            if(this->dType == DataType::DOB){ // I have realized that size checking in the init function for the output is kind of stupid so I removed it. Maybe I am wrong
                std::vector<double> tData;
                tData.reserve(zOut.size());
                for(size_t i; i < zOut.size(); i++){
                    tData.push_back(0.0);
                }
                this->sensedLiftDOB->set(this->zOut, tData);
            } else if(this->dType == DataType::VEC){
                //Does not work without an input because there is no way to acertain the dimension of the vectors. 
                std::cout << "WARNING: cannot initialize a vector without a connected input. Things might break"; // Because we use set in the step function then this might not be an issue. Still the warning is appropriate. Also having the Data pre-set by the orchestrator might solve a lot of these issues
            } else {
                throw std::runtime_error("Invalid type");
            }
        } else if(this->sLiftConnected && this->rLiftConnected) { //Input and output both connected
            std::cout << "WARNING: This module must be initialized after the previous module for predictable results."; // eventually maybe I should add some sort of logic to the whole project to deal with these issues but for now it is whatever.
            if(this->dType == DataType::DOB){ // Im not sure if I should 0-init as well or just pass through the input which would require the module to be initialized after the previous module.
                this->sensedLiftDOB->set(this->zOut, uasisi::interpolate(this->realLiftDOB->coords(), this->realLiftDOB->SWData(), this->zOut, getInterpType(this->iType)));
            } else if(this->dType == DataType::VEC){
                this->sensedLiftVEC->set(this->zOut, uasisi::interpolate(this->realLiftVEC->coords(), this->realLiftVEC->SWData(), this->zOut, getInterpType(this->iType)));
            } else {
                throw std::runtime_error("Invalid type");
            }
        } else {
            throw std::runtime_error("Module not connected properly");
        }
    } else {
        throw std::runtime_error("Module is not yet connected. Please validate before initalizing");
    }
}

std::vector<SignalInfo> PerfectSen::declareSignals(){
    if(!this->dTypeIsSet){
        throw std::runtime_error("Data type is not set");
    }
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(2);
    reqSignals.push_back(SignalInfo("realLift", this->dType, SignalType::IN));
    reqSignals.push_back(SignalInfo("sensedLift", this->dType, SignalType::OUT));

    return reqSignals;
}

void PerfectSen::validateConnections(){
    
    if(!this->dTypeIsSet){
        throw std::runtime_error("Validation failed. Make sure dType is set");
    }
    if(!((dType == DataType::DOB && this->realLiftDOB && this->sensedLiftDOB) || (dType == DataType::VEC && this->realLiftVEC && this->sensedLiftVEC))){
        std::cout << "WARNING: some signals might be connected\n";
        if(this->realLiftDOB || this->realLiftVEC){
            if(this->inputs.size() != 1 || this->inputs[0].name() != "sensedLift"){ 
                throw std::runtime_error("Validation failed");
            }
        } else if (this->sensedLiftDOB || this->sensedLiftVEC){
            if(this->outputs.size() != 1 || this->outputs[0].name() != "realLift"){ 
                throw std::runtime_error("Validation failed");
            }
        }
    } else {
        if(this->outputs.size() != 1 || this->inputs.size() != 1 || this->outputs[0].name() != "sensedLift" || this->inputs[0].name() != "realLiftVEC"){ 
            throw std::runtime_error("Validation failed");
        }
    }
    if(dType == DataType::DOB){
        if(realLiftDOB) this->rLiftConnected = true;
        if(sensedLiftDOB){
            if(!this->zOutIsSet || this->sensedLiftDOB->size() != this->zOut.size()){
                throw std::runtime_error("Output size mismatch"); 
            }
            this->sLiftConnected = true;
        }
    } else if(dType == DataType::VEC){
        if(realLiftVEC) this->rLiftConnected = true;
        if(sensedLiftVEC){
            if(!this->zOutIsSet || this->sensedLiftVEC->size() != this->zOut.size()){
                throw std::runtime_error("Output size mismatch"); 
            }
            this->sLiftConnected = true;
        }
    } else {
        throw std::runtime_error("Invalid data type");
    }
    
    this->isConnected = true;
}

void PerfectSen::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "sensedLift"){
        this->sensedLiftDOB = x;
        this->outputs.push_back(SignalInfo(name, DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectSen::connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x){

    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "sensedLift"){
        this->sensedLiftVEC = x;
        this->outputs.push_back(SignalInfo(name, DataType::VEC, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectSen::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){

    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realLift"){
        this->realLiftDOB = x;
        this->inputs.push_back(SignalInfo(name, DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectSen::connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x){

    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realLift"){
        this->realLiftVEC = x;
        this->inputs.push_back(SignalInfo(name, DataType::VEC, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectSen::step(double t, double dt){
    
    if(!this->isSet){
        throw std::runtime_error("Module not initialized");
    }
    if(!this->dTypeIsSet || !this->iTypeIsSet){
        throw std::runtime_error("Error, Perfect Actuator Module not fully setup");
    }
    if(this->isConnected){
        if(!this->sLiftConnected){
            //nothing
        } else if(!this->rLiftConnected){
            //nothing, no input, no change. Maybe it would be better to set to 0 to be realistic but I don't think it would be very practical.
        } else if(this->sLiftConnected && this->rLiftConnected){
            if(this->dType == DataType::DOB){ // Im not sure if I should 0-init as well or just pass through the input which would require the module to be initialized after the previous module.
                this->sensedLiftDOB->set(this->zOut, uasisi::interpolate(this->realLiftDOB->coords(), this->realLiftDOB->SWData(), this->zOut, getInterpType(this->iType)));
            } else if(this->dType == DataType::VEC){
                this->sensedLiftVEC->set(this->zOut, uasisi::interpolate(this->realLiftVEC->coords(), this->realLiftVEC->SWData(), this->zOut, getInterpType(this->iType)));
            } else {
                throw std::runtime_error("Invalid type");
            }
        } else {
            throw std::runtime_error("Module not properly connected");
        }
    } else {
        throw std::runtime_error("Module is not yet connected. Please validate before initalizing")
    }

}

void PerfectSen::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void PerfectSen::setZOut(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->zOut = zNew;
    this->zOutIsSet = true;
}

void PerfectSen::setDType(const DataType& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->dTypeIsSet){
        throw std::runtime_error("Data type already set");
    }
    if(tNew == DataType::AIR){
        throw std::runtime_error("Airfoil data not supported");
    }
    this->dType = tNew;
    this->dTypeIsSet = true;
}

void PerfectSen::setIType(const interpType& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = tNew;
    this->iTypeIsSet = true;
}

}

namespace{
static uasisi::ModuleRegistration<uasisi::PerfectSen> registrationPerfectSen("perfectSen");
}
