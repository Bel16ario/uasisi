#include "constantAccelAct.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace uasisi {

ConstantAccelAct::ConstantAccelAct(){
 std::cout << "Constant Acceleration Actuator created\n";
}
// I really should implement a method for adding thickness. I remember in the perfect act module I dint want to because I felt like it was a little akward with respect to the method return type (void vs vector).
void ConstantAccelAct::init(const Config& config){
    
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(this->thetaIsSet && this->omegaIsSet){
        this->initialStateIsSet = true;
    } else {
        throw std::runtime_error("Initial actuator state must be set");
    }
    if(!this->dTypeIsSet || !this->iTypeIsSet || !this->maxPosIsSet || !this->maxVelIsSet || !this->maxAccelIsSet || !this->centerPosSet){
        throw std::runtime_error("Error, Constant Acceleration Actuator Module not fully setup");
    }
    if(this->isConnected){
        if(this->rGeometryConnected){
            if(this->initialStateIsSet){
                if(this->dType == DataType::DOB && this->z.size() == this->targetGeometryDOB->size()){
                    if(this->addThickness){
                        if(!this->thicknessIsSet || this->thickness > 0 || this->z.size() < 3){
                            throw std::runtime_error("Module not configured for thickness");
                        }
                        std::vector<double> zThick;
                        std::vector<double> dataThick;

                        double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                        double zTemp = this->z[0];

                        zThick.reserve(2*this->z.size());
                        dataThick.reserve(2*this->z.size());
                        zThick.push_back(this->z[0]);
                        dataThick.push_back(this->initialStateDOB[0]);
                        zThick.push_back(zThick[0] + this->thickness);
                        dataThick.push_back(this->initialStateDOB[0]);

                        for(size_t i = 1; i < (z.size() - 1); i++){
                            zTemp += dz;
                            zThick.push_back(zTemp - this->thickness/2);
                            dataThick.push_back(this->initialStateDOB[i]); //Check size fit on setter method
                            zThick.push_back(zTemp + this->thickness/2);
                            dataThick.push_back(this->initialStateDOB[i]);
                        }

                        zThick.push_back(z[this->z.size()-1] - this->thickness);
                        zThick.push_back(z[this->z.size()-1]);
                        dataThick.push_back(this->initialStateDOB[this->z.size() - 1]);
                        dataThick.push_back(this->initialStateDOB[this->z.size() - 1]);

                        this->realGeometryDOB->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType)));
                    } else {
                        this->realGeometryDOB->set(zOut, uasisi::interpolate(this->z, this->initialStateDOB, this->zOut, getInterpType(this->iType)));
                    }
                } else throw std::runtime_error("Invalid type or initial state size mismatch");
            } else { //Kind of undefined behaviour for the first step. The orchestrator should not step modules that depend on this one until this one has been stepped
                throw std::runtime_error("At the moment, this module requires an initial state to be set"); 
            }
        }
    } else {
        throw std::runtime_error("Module unconnected");
    }

    this->isSet = true;

}

std::vector<SignalInfo> ConstantAccelAct::declareSignals(){
    if(!this->dTypeIsSet){
        throw std::runtime_error("Data type is not set");
    }
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(2);
    reqSignals.push_back(SignalInfo("targetGeometry", this->dType, SignalType::IN));
    reqSignals.push_back(SignalInfo("realGeometry", this->dType, SignalType::OUT));

    return reqSignals;
}

void ConstantAccelAct::validateConnections(){
    
    if(!this->dTypeIsSet){
        throw std::runtime_error("Validation failed. Make sure dType is set");
    }
    if(!(dType == DataType::DOB && this->targetGeometryDOB && this->realGeometryDOB)){
        std::cout << "WARNING: some signals might be unconnected\n";
        if(this->targetGeometryDOB){ //Only input connected
            if(this->inputs.size() != 1 || this->inputs[0].name() != "targetGeometry"){ 
                throw std::runtime_error("Validation failed");
            }
        } else if (this->realGeometryDOB){ // Only output connected
            if(this->outputs.size() != 1 || this->outputs[0].name() != "realGeometry"){ 
                throw std::runtime_error("Validation failed");
            }
        }
    } else {
        if(this->outputs.size() != 1 || this->inputs.size() != 1 || this->outputs[0].name() != "realGeometry" || this->inputs[0].name() != "targetGeometry"){ 
            throw std::runtime_error("Validation failed");
        }
    }
    if(dType == DataType::DOB){//Again, it falls on the orchestrator to resize the vector passing the pointer
        if(targetGeometryDOB){
            if(!this->zIsSet || this->targetGeometryDOB->size() != this->z.size()){
                throw std::runtime_error("Input size mismatch");
            }
            this->tGeometryConnected = true;
        }
        if(realGeometryDOB){
            if(!this->zOutIsSet || this->realGeometryDOB->size() != this->zOut.size()){
                throw std::runtime_error("Output size mismatch"); 
            }
            this->rGeometryConnected = true;
        }
    } else {
        throw std::runtime_error("Invalid data type");
    }
    this->isConnected = true;

}

void ConstantAccelAct::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realGeometry"){
        this->realGeometryDOB = x;
        this->outputs.push_back(SignalInfo(name, DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void ConstantAccelAct::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){ //Does the const really prevent the module from writing to the SWVec? not sure

    if(!this->dTypeIsSet || this->dType != DataType::DOB){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryDOB = x;
        this->inputs.push_back(SignalInfo(name, DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void ConstantAccelAct::step(double t, double dt){ // I honestly think like these modules should have a lot more methods to make the important methods like init, validate or step a lot more readable. I will take it into account for the next modules.
    if(this->tGeometryConnected && this->rGeometryConnected){
        if(this->dType == DataType::DOB){
            double posError;
            double maxDeltaTheta;
            double minDeltaTheta;
            double deltaOmega;
            double deltaTheta;
            double stopDistance;
            for(size_t i = 0; i < this->z.size(); i++){
                if(!((*this->targetGeometryDOB)[i] >= this->centerPos[i] + this->maxPos[i] || (*this->targetGeometryDOB)[i] <= this->centerPos[i] - this->maxPos[i])){
                    posError = (*this->targetGeometryDOB)[i] - this->theta[i];
                    if(std::abs(posError) < this->posTol){
                        this->omega[i] = 0.0;
                        continue;
                    }
                    maxDeltaTheta = dt*this->omega[i] + 0.5*this->maxAccel[i];
                    minDeltaTheta = dt*this->omega[i] - 0.5*this->maxAccel[i];
                    stopDistance = std::abs((this->omega[i]*this->omega[i]*0.5)/this->maxAccel[i]);
                    if(stopDistance >= std::abs(posError)){
                        deltaOmega = -std::copysign(std::min(this->maxAccel[i]*dt, std::abs(this->omega[i])), this->omega[i])
                    } else {
                        deltaOmega = std::copysign(std::min(this->maxAccel[i]*dt, this->maxVel[i] - std::abs(this->omega[i])), std::abs(posError));
                    }
                    this->omega[i] += deltaOmega;
                    deltaTheta = this->omega[i]*dt;
                    if(std::abs(deltaTheta) > (std::abs(posError) + 2*this->posTol)){
                        std::cout << "Actuator instability\n";
                        deltaTheta = 0.0;
                        this->omega[i] = 0.0;
                    }
                    this->theta[i] += deltaTheta;
                } else if((*this->targetGeometryDOB)[i] > this->centerPos[i] + this->maxPos[i]){
                    this->theta[i] = this->centerPos[i] + this->maxPos[i];
                } else if((*this->targetGeometryDOB)[i] < this->centerPos[i] - this->maxPos[i]){
                    this->theta[i] = this->centerPos[i] - this->maxPos[i];
                }
            }
            if(this->addThickness){
                if(!this->thicknessIsSet || this->thickness > 0 || this->z.size() < 3){
                    throw std::runtime_error("Module not configured for thickness")
                }
                std::vector<double> zThick;
                std::vector<double> dataThick;

                double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                double zTemp = this->z[0];

                zThick.reserve(2*this->z.size());
                dataThick.reserve(2*this->z.size());
                zThick.push_back(this->z[0]);
                dataThick.push_back(this->theta[0]);
                zThick.push_back(zThick[0] + this->thickness);
                dataThick.push_back(this->theta[0]);

                for(size_t i = 1; i < (z.size() - 1); i++){
                    zTemp += dz;
                    zThick.push_back(zTemp - this->thickness/2);
                    dataThick.push_back(this->theta[i]);
                    zThick.push_back(zTemp + this->thickness/2);
                    dataThick.push_back(this->theta[i]);
                }

                zThick.push_back(z[this->z.size()-1] - this->thickness);
                zThick.push_back(z[this->z.size()-1]);
                dataThick.push_back(this->theta[this->z.size() - 1]);
                dataThick.push_back(this->theta[this->z.size() - 1]);

                this->realGeometryDOB->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType))); 
            } else {
                this->realGeometryDOB->set(zOut, uasisi::interpolate(this->z, this->theta, this->zOut, getInterpType(this->iType)));
            }
        } else {
            throw std::runtime_error("Invalid data type");
        }
    }

}

void ConstantAccelAct::setZ(const std::vector<double>& zNew){ //I just realised that this could be obtained from the config at init. I don't want to mess around with it right now, I wrote the config code long ago but still worth keeping in mind. The orchestrator will have to do it in the helper function. 
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->zIsSet){
        throw std::runtime_error("To avoid size mismatch errors. It is prohibited to change z after it has been already set");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void ConstantAccelAct::setZOut(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->zOut = zNew;
    this->zOutIsSet = true;
}

void ConstantAccelAct::setMaxPos(const std::vector<double>& thetaMax){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet){
        throw std::runtime_error("Actuator position must be set first");
    }
    if(this->z.size() != thetaMax.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(double val : thetaMax){ //This is sadly necessary. I really should validate z and zOut but oh well
        if(val < 0 || val > 2*M_PI){ //really the limit should be something like 25 degrees. Not sure if I should set an arbitrary ceiling.
            throw std::runtime_error("Invalid vector. Bad angle found");
        }
    }
    this->maxPos = thetaMax;
    this->maxPosIsSet = true;
}

void ConstantAccelAct::setMaxVel(const std::vector<double>& omegaMax){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet){
        throw std::runtime_error("Actuator position must be set first");
    }
    if(this->z.size() != omegaMax.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(double val : omegaMax){//Is there any other necessary check? 
        if(val < 0){
            throw std::runtime_error("Invalid vector. Negative value found");
        }
    }
    this->maxVel = omegaMax;
    this->maxVelIsSet = true;
}

void ConstantAccelAct::setMaxAccel(const std::vector<double>& alphaMax){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet){
        throw std::runtime_error("Actuator position must be set first");
    }
    if(this->z.size() != alphaMax.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(double val : alphaMax){
        if(val < 0){
            throw std::runtime_error("Invalid vector. Negative value found");
        }
    }
    this->maxAccel = alphaMax;
    this->maxAccelIsSet = true;
}

void ConstantAccelAct::setCenterPos(const std::vector<double>& thetaCenter){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet || !this->maxPosIsSet){
        throw std::runtime_error("Actuator position and maximum position vectors must be set first");
    }
    if(this->z.size() != thetaCenter.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(size_t i = 0; i < this->z.size(); i++){
        if(thetaCenter[i] - this->maxPos[i] < -2*M_PI || thetaCenter[i] + this->maxPos[i] > 2*M_PI){ //Same deal. Maybe the deal here would be to require maxPos to be set first and check the sum? Or should it be the other way around?. I think it should be the other way around but whatever.
            throw std::runtime_error("Invalid vector. Bad angle found");
        }
    }
    this->centerPos = thetaCenter;
    this->centerPosIsSet = true;
}

void ConstantAccelAct::setTheta(const std::vector<double>& thetaNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet || !this->maxPosIsSet || !this->centerPosIsSet){
        throw std::runtime_error("Actuator position, maximum position and center position vectors must be set first");
    }
    if(this->z.size() != thetaNew.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(size_t i = 0; i < this->z.size(); i++){
        if(thetaNew[i] < this->centerPos[i] - this->maxPos[i] || thetaNew[i] > thetaCenter[i] + this->maxPos[i]){
            throw std::runtime_error("Invalid vector. Angle out of range found");
        }
    }
    this->theta = thetaNew;
    this->thetaIsSet = true;
}

void ConstantAccelAct::setOmega(const std::vector<double>& omegaNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet){
        throw std::runtime_error("Actuator position vector must be set first");
    }
    if(this->z.size() != thetaNew.size()){
        throw std::runtime_error("Size mismatch");
    }
    for(size_t i = 0; i < this->z.size(); i++){
        if(omegaNew[i] < - this->maxVel[i] || omegaNew[i] > this->maxVel[i]){
            throw std::runtime_error("Invalid vector. Angle out of range found");
        }
    }
    this->omega = omegaNew;
    this->omegaIsSet = true;
}

void ConstantAccelAct::setPosTol(const double& tolNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(tolNew < 0){
        throw std:.runtime_error("Tolerance must be set as a positive double");
    }
    this->thickness = tNew;
    this->zIsSet = true;
}

void ConstantAccelAct::setThickness(const double& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->thickness = tNew;
    this->zIsSet = true;
}

void ConstantAccelAct::setIType(const interpType& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = tNew;
    this->iTypeIsSet = true;
}

}

namespace {
static uasisi::ModuleRegistration<uasisi::ConstantAccelAct> registrationConstantAccelAct("constantAccelAct");
}
