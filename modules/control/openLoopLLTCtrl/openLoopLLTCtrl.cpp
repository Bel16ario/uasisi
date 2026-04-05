#include "openLoopLLTCtrl.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace uasisi{

double trapz(const std::vector<double>& x, const std::vector<double>& y);
std::vector<double> threePointDiff(const std::vector<double>& x, const std::vector<double>& y);

OpenLoopLLTCtrl::OpenLoopLLTCtrl(){

    std::cout << "OpenLoopLLT control created\n";

}

void OpenLoopLLTCtrl::init() {

    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    bool dataIsSet = (this->vInfIsSet && this->rhoIsSet && this->chordsIsSet && this->zlAngleIsSet && this->clSlopeIsSet && this->thetaMaxIsSet && this->thetaCenterIsSet && this->spanIsSet);
    if(!this->zIsSet || !dataIsSet || !this->dTypeIsSet || !this->iTypeIsSet || !this->actCoordsAreSet){
        throw std::runtime_error("Error, Open Loop LLT control Module not fully setup");
    }
    if(!this->isConnected){
        throw std::runtime_error("Module connections have not been validated yet");
    }

    if(this->actCoords.size() > this->z.size() || this->z.size() < 4 || this->z.size() % 2){
        throw std::runtime_error("Coordinate Error");
    }
    if(this->z.size() != this->chords.size() || this->z.size() != this->zlAngle.size() || this->z.size() != this->clSlope.size() || this->z.size() != this->thetaMax.size() || this->z.size() != this->thetaCenter.size()){
        throw std::runtime_error("Size mismatch");
    }
    if(this->z.size() != this->targetLift->size()){
        throw std::runtime_error("Size mismatch");
    }
    if(this->vInf < 0 || this->rho <= 0 || this->span <= 0){
        throw std::runtime_error("Invalid values found");
    }
    for(size_t i = 1; i < this->z.size(); i++){
        if(this->z[i] < this->z[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    for(size_t i = 1; i < this->actCoords.size(); i++){
        if(this->actCoords[i] < this->actCoords[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->theta.resize(this->z.size());
    this->computeK2();
    this->computeTheta();

    this->isSet = true;

}

std::vector<SignalInfo> OpenLoopLLTCtrl::declareSignals(){
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(1);
    reqSignals.push_back(SignalInfo("targetLift", DataType::DOB, SignalType::IN));
    reqSignals.push_back(SignalInfo("targetGeometry", DataType::DOB, SignalType::OUT));

    return reqSignals;
}

void OpenLoopLLTCtrl::validateConnections(){
    
    if(!this->targetGeometryDOB){
        std::cout << "WARNING: output unconnected\n";
    } else {
        if(this->outputs.size() != 1 || this->outputs[0].name() != "targetGeometry"){ 
            throw std::runtime_error("Validation failed");
        }
    }
    if(!this->targetLift){
        std::cout << "WARNING: input unconnected\n";
    } else {
        if(this->inputs.size() != 1 || this->inputs[0].name() != "targetLift"){ 
            throw std::runtime_error("Validation failed");
        }
    }
    this->isConnected = true;

}

void OpenLoopLLTCtrl::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

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

void OpenLoopLLTCtrl::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetLift"){
        this->targetLift = x;
        this->inputs.push_back(SignalInfo("targetLift", DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void OpenLoopLLTCtrl::step(double t, double dt){
    if(!this->isSet || !this->isConnected){
        throw std::runtime_error("Module not initalized");
    }
    this->computeTheta();
}

void OpenLoopLLTCtrl::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void OpenLoopLLTCtrl::setActCoords(const std::vector<double>& zNew){ //Should check for valid coords at some point
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->actCoords = zNew;
    this->actCoordsAreSet = true;
}

void OpenLoopLLTCtrl::setVInf(double vInfNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module Already connected");
    }
    this->vInf = vInfNew;
    this->vInfIsSet = true;
}

void OpenLoopLLTCtrl::setRho(double rhoNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module Already connected");
    }
    this->rho = rhoNew;
    this->rhoIsSet = true;
}

void OpenLoopLLTCtrl::setSpan(double spanNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module Already connected");
    }
    this->span = spanNew;
    this->spanIsSet = true;
}

void OpenLoopLLTCtrl::setChords(const std::vector<double>& chordsNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->chords = chordsNew;
    this->chordsIsSet = true;
}

void OpenLoopLLTCtrl::setZeroLiftAngle(const std::vector<double>& zlAngleNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->zlAngle = zlAngleNew;
    this->zlAngleIsSet = true;
}

void OpenLoopLLTCtrl::setClSlope(const std::vector<double>& clSlopeNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->clSlope = clSlopeNew;
    this->clSlopeIsSet = true;
}

void OpenLoopLLTCtrl::setThetaMax(const std::vector<double>& thetaMaxNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->thetaMax = thetaMaxNew;
    this->thetaMaxIsSet = true;
}

void OpenLoopLLTCtrl::setThetaCenter(const std::vector<double>& thetaCenterNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->thetaCenter = thetaCenterNew;
    this->thetaCenterIsSet = true;
}

void OpenLoopLLTCtrl::setIType(const interpType& t){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = t;
    this->iTypeIsSet = true;
}

void OpenLoopLLTCtrl::computeTheta(){
    std::vector<double> circ, k1, dCirc;
    circ.resize(this->z.size());
    k1.resize(this->z.size());
    for(size_t i = 0; i < this->z.size(); i++){
        circ[i] = (*this->targetLift)[i] / (this->rho * this->vInf);
        k1[i] = 2*circ[i] / (this->clSlope[i]*this->vInf * this->chords[i]);
    }
    dCirc = threePointDiff(this->z, circ);
    double itgrl = 0;
    for(size_t i = 0; i < this->z.size(); i++){
        this->theta[i] = this->k1[i] + this->zlAngle[i] + this->k2*itgrl;
    }
}

void OpenLoopLLTCtrl::computeK2(){
    this->k2 = 1 / (4*M_PI*this->vInf);
}

std::vector<double> threePointDiff(const std::vector<double>& x, const std::vector<double>& y){
    if(x.size() != y.size() || x.size() < 3){
        throw std::runtime_error("Size mismatch");
    }
    for(size_t i = 1; i < x.size(); i++){
        if(x[i] < x[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    std::vector<double> result;
    result.resize(x.size());
    for(size_t i = 1; i < x.size() - 1; i++){
        result[i] = (1 / (x[i+1] - x[i-1]))*(y[i+1] - y[i-1]);
    }
    result[0] = (1 / (x[2] - x[0]))*(-3*y[0] + 4*y[1] - y[2]);
    result.back() = (1 / (x[result.size()-1] - x[result.size()-3]))*(3*y[result.size()-1] - 4*y[result.size()-2] + y[result.size()-3]);
    return result;
}

double trapz(const std::vector<double>& x, const std::vector<double>& y){

    if(x.size() != y.size() || x.size() < 4){
        throw std::runtime_error("vector size error");
    }
    double sum = 0.0;
    for(size_t i = 1; i < x.size(); i++){
        sum += 0.5 * (y[i] + y[i-1]) * (x[i] - x[i-1]);
    }
    return sum;

}

}

namespace{
static uasisi::ModuleRegistration<uasisi::OpenLoopLLTCtrl> registrationOpenLoopLLTCtrl("openLoopLLTCtrl");
}
