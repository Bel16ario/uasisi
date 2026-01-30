#include "constantCtrl.hpp"
#include <iostream>
#include <stdexcept>

namespace uasisi{

ConstantCtrl::ConstantCtrl(){

    std::cout << "Constant Control created\n";

}

void ConstantCtrl::init(const Config& config) {

    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->zIsSet || !this->dataIsSet || !this->dTypeIsSet || !this->iTypeIsSet){
        throw std::runtime_error("Error, Constant Control Module not fully setup");
    }
    if(this->isConnected){
        if(dType == DataType::DOB){
            if(z.size() != sData.size()){
                throw std::runtime_error("Input size mismatch");
            }
            if(this->targetGeometryDOB->size() != config.getActCoords().size()) throw std::runtime_error("Output size mismatch");
            this->targetGeometryDOB->set(config.getActCoords(), uasisi::interpolate(this->z, this->sData, config.getActCoords(), getInterpType(this->iType)));
        } else if(dType == DataType::VEC){
            if(z.size() != vData.size()){
                throw std::runtime_error("Input size mismatch");
            }
            if(this->targetGeometryVEC->size() != config.getActCoords().size()) throw std::runtime_error("Output size mismatch");
            this->targetGeometryVEC->set(config.getActCoords(), uasisi::interpolate(this->z, this->vData, config.getActCoords(), getInterpType(this->iType)));
        } else if(dType == DataType::AIR){
            if(z.size() != aData.size()){
                throw std::runtime_error("Input size mismatch");
            }
            if(this->targetGeometryAIR->size() != config.getActCoords().size()) throw std::runtime_error("Output size mismatch");
            this->targetGeometryAIR->set(config.getActCoords(), uasisi::interpolate(this->z, this->aData, config.getActCoords(), getInterpType(this->iType)));
        } else {
            throw std::runtime_error("Invalid type");
        }
    } else {
        throw std::runtime_error("Module unconnected");
    }

    this->isSet = true;

}

std::vector<SignalInfo> ConstantCtrl::declareSignals(){
    if(!this->dTypeIsSet){
        throw std::runtime_error("Data type is not set");
    }
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(1);
    reqSignals.push_back(SignalInfo("targetGeometry", this->dType, SignalType::OUT));

    return reqSignals;
}

void ConstantCtrl::validateConnections(){
    
    if(!this->dTypeIsSet){
        throw std::runtime_error("Validation failed. Make sure dType is set");
    }
    if(!((dType == DataType::DOB && this->targetGeometryDOB) || (dType == DataType::VEC && this->targetGeometryVEC) || (dType == DataType::AIR && this->targetGeometryAIR))){
        std::cout << "WARNING: output unconnected\n";
    } else {
        if(this->outputs.size() != 1 || this->outputs[0].name() != "targetGeometry"){ 
            throw std::runtime_error("Validation failed");
        }
        this->isConnected = true;
    }

}

void ConstantCtrl::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryDOB = x;
        this->outputs.push_back(SignalInfo(name, DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void ConstantCtrl::connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x){

    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryVEC = x;
        this->outputs.push_back(SignalInfo(name, DataType::VEC, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void ConstantCtrl::connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x){
    
    if(!this->dTypeIsSet || this->dType != DataType::AIR){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryAIR = x;
        this->outputs.push_back(SignalInfo(name, DataType::AIR, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void ConstantCtrl::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void ConstantCtrl::setSData(const std::vector<double>& sDataNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Wrong or unset data type");
    }
    this->sData = sDataNew;
    this->dataIsSet = true;
}

void ConstantCtrl::setVData(const std::vector<std::vector<double>>& vDataNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Wrong or unset data type");
    }
    this->vData = vDataNew;
    this->dataIsSet = true;
}

void ConstantCtrl::setAData(const std::vector<airfoil>& aDataNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::AIR){
        throw std::runtime_error("Wrong or unset data type");
    }
    this->aData = aDataNew;
    this->dataIsSet = true;
}

void ConstantCtrl::setDType(const DataType& t){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->dTypeIsSet){
        throw std::runtime_error("Data type already set and cannot be changed. Please instantiate a new module");
    }
    this->dType = t;
    this->dTypeIsSet = true;
}

void ConstantCtrl::setIType(const interpType& t){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = t;
    this->iTypeIsSet = true;
}

}

namespace{
static uasisi::ModuleRegistration<uasisi::ConstantCtrl> registrationConstantCtrl("constantCtrl");
}
