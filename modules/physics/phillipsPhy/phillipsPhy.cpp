//TODO: Implement obtainPolars() with xfoil and also add number of points and force angular distribution.

#include "phillipsPhy.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uasisi{

PhillipsPhy::PhillipsPhy(){
    std::cout << "Phillips' LLT Physics module created\n";
}

void PhillipsPhy::init(){

    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    } // I don't need to check for initial state I think because refresh methods already take care of it
    if(!this->iTypeIsSet || !this->fCondsIsSet || !this->zOutIsSet){
        throw std::runtime_error("Module has not been fully set-up yet");
    }
    if(!this->isConnected){
        throw std::runtime_error("Module connections have not been validated yet");
    }
    if((this->rGeometryIsConnected || this->rTwistIsConnected) && (this->rLiftIsConnected || this->rTLiftIsConnected || this->rRMomentIsConnected)){ // Input and output connected
    
        if(!this->rGeometryIsConnected && !this->foilIsSet){//Provide airfoils
            this->interpGeo.resize(this->zOut.size() - 2);
            for(airfoil& val : this->interpGeo){
                val.setName(this->defaultFoil);
            }
            this->foilIsSet = true;
        }
        if(!this->rTwistIsConnected && !this->twistIsSet){//Provide twist
            this->interpTwist.resize(this->zOut.size()-2);
            std::fill(this->interpTwist.begin(), this->interpTwist.end(), this->defaultTwist);
            this->twistIsSet = true;
        }
        if(!this->chordIsSet){//Provide chord 
            this->chord.resize(this->zOut.size()-2);
            std::fill(this->chord.begin(), this->chord.end(), this->defaultChord);
            this->chordIsSet = true;
        }
        this->refreshZ();
        this->refreshData();
        this->translateDomain();
        this->computeRe();
        this->obtainPolars();
        if(!this->alpha0IsSet){
            this->alpha0 = Eigen::VectorXd::Zero(this->phi.size());
            this->alpha0IsSet = true;
        }
        this->generateFij(); //A lot of redundancy in the init. I will leave it for the sake of my sanity. When stepping I think it is fairly efficient which is nice.
        this->computeLift();

    } else if(this->rGeometryIsConnected || this->rTwistIsConnected){ //Only input connected
        std::cout << "WARNING: no output connected. Module will IDLE. Connect monitor to force computation\n";
    } else if(this->rLiftIsConnected || this->rTLiftIsConnected || this->rRMomentIsConnected){ //Only output connected
        throw std::runtime_error("No input connected. Cannot compute");
    } else { //No input or output
        std::cout << "WARNING: No input or output. Module will IDLE\n";
    }
    this->isSet = true;

}

std::vector<SignalInfo> PhillipsPhy::declareSignals(){
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(5);
    reqSignals.push_back(SignalInfo("realGeometry", DataType::AIR, SignalType::IN));
    reqSignals.push_back(SignalInfo("realGeometry", DataType::DOB, SignalType::IN));
    reqSignals.push_back(SignalInfo("realLift", DataType::DOB, SignalType::OUT));
    reqSignals.push_back(SignalInfo("realTotalLift", DataType::SCA, SignalType::OUT));
    reqSignals.push_back(SignalInfo("realRollingMoment", DataType::SCA, SignalType::OUT));
    return reqSignals;
}

void PhillipsPhy::validateConnections(){

    if(!this->zOutIsSet){
        throw std::runtime_error("zOut must be set");
    }
    if(std::abs(this->zOut.back() - this->zOut[0] - this->span) > this->tolerance){
        throw std::runtime_error("Span and zOut do not match");
    }
    size_t inputCount = bool(this->realTwist) + bool(this->realGeometry);
    size_t outputCount = bool(this->realLift) + bool(this->realTotalLift) + bool(this->realRollingMoment);
    if(this->inputs.size() != inputCount || this->outputs.size() != outputCount){
        throw std::runtime_error("Unexpected error with signal ledgers");
    }
    if(inputCount != 2){
        std::cout << "WARNING: some inputs might be unconnected\n";
    }
    if(outputCount != 3){
        std::cout << "WARNING: some outputs might be unconnected\n";
    }
    if(this->realTwist){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "realGeometry" && s.dataType() == DataType::DOB;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
        this->rTwistIsConnected = true;
    }
    if(this->realGeometry){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "realGeometry" && s.dataType() == DataType::AIR;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
        this->rGeometryIsConnected = true;
    }
    if(this->realLift){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realLift" && s.dataType() == DataType::DOB;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
        this->rLiftIsConnected = true;
    }
    if(this->realTotalLift){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realTotalLift" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
        this->rTLiftIsConnected = true;
    }
    if(this->realRollingMoment){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realRollingMoment" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
        this->rRMomentIsConnected = true;
    }
    this->isConnected = true;

}

void PhillipsPhy::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realLift"){
        this->realLift = x;
        this->outputs.push_back(SignalInfo("realLift", DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PhillipsPhy::connectOutputScalar(const std::string& name, double* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realTotalLift"){
        this->realTotalLift = x;
        this->outputs.push_back(SignalInfo("realTotalLift", DataType::SCA, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else if(name == "realRollingMoment") {
        this->realRollingMoment = x;
        this->outputs.push_back(SignalInfo("realRollingMoment", DataType::SCA, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PhillipsPhy::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realGeometry"){
        this->realTwist = x;
        this->inputs.push_back(SignalInfo("realGeometry", DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PhillipsPhy::connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realGeometry"){
        this->realGeometry = x;
        this->inputs.push_back(SignalInfo("realGeometry", DataType::AIR, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PhillipsPhy::step(double t, double dt){ //For complex airfoil variation, the orchestrator must manually call for Re to be computed and polars to be reobtained.
    if(!this->isSet || !this->isConnected){
        throw std::runtime_error("Module is not initialized/validated");
    }
    
    if((this->rGeometryIsConnected || this->rTwistIsConnected) && (this->rLiftIsConnected || this->rTLiftIsConnected || this->rRMomentIsConnected)){ // Input and output connected
    
        this->refreshData(); //Z is only refreshed manually
        this->computeLift();
    
    } else if(this->rGeometryIsConnected || this->rTwistIsConnected){ //Only input connected
        std::cout << "WARNING: no output connected. Module will IDLE. Connect monitor to force computation\n";
    } else if(this->rLiftIsConnected || this->rTLiftIsConnected || this->rRMomentIsConnected){ //Only output connected
        throw std::runtime_error("No input connected. Cannot compute");
    } else { //No input or output
        std::cout << "WARNING: No input or output. Module will IDLE\n";
    }
    

}

void PhillipsPhy::computeRe(){
    if(!this->chordIsSet){
        throw std::runtime_error("Chords need to be set before computing Reynolds' numbers");
    }
    if(!this->zOutIsSet){
        throw std::runtime_error("zOut needs to be set before computing Reynolds' numbers");
    }
    if(this->zOut.size() - 2 != this->chord.size()){
        throw std::runtime_error("Size mismatch");
    }
    this->Re.reserve(this->zOut.size() - 2);
    for(double val : this->chord){
        this->Re.push_back(this->fConds.getDensity() * this->fConds.getVelocity().magnitude() * val / this->fConds.getViscosity());
    }
    this->ReIsSet = true;
}

void PhillipsPhy::obtainPolars(){//For dev purposes just return constant for now

    if(!this->zOutIsSet || !this->ReIsSet || !this->foilIsSet){
        throw std::runtime_error("zOut, foil and Re must be set");
    }
    for(airfoil& val : this->interpGeo){
        if(val.size() == 0){
            val.generate(this->nPoints);
        }
    }
    this->lSlope.resize(this->zOut.size() - 2);
    this->alpha0L = Eigen::VectorXd::Zero(this->zOut.size() - 2);
    for(size_t i = 0; i < this->lSlope.size(); i++){
        this->lSlope[i] = 6.5317;
        this->alpha0L(i) = -0.0375;
    }
    this->polarsAreSet = true;

}

void PhillipsPhy::translateDomain(){

    if(!this->zOutIsSet || this->zOut.size() < 4){// the two tip points will be removed
        throw std::runtime_error("zOut vector needs to be set before computing the domain");
    }
    if(this->PhiIsSet){
        throw std::runtime_error("Phi vector is already set");
    }
    this->phi.reserve(this->zOut.size() - 2);
    double zCenter = (this->zOut[0] +  this->zOut.back()) * 0.5; //goes without saying that the coordinates should be ordered
    double r = std::abs(this->zOut.back() - zCenter);
    double adj;
    for(size_t i = 1; i < this->zOut.size() - 1; i++ ){
        adj = this->zOut[i] - zCenter;
        this->phi.push_back(std::acos(adj/r));
    }
    this->PhiIsSet = true;

}

void PhillipsPhy::generateFij(){//
    if(!this->zOutIsSet || !this->polarsAreSet || !this->spanIsSet || !this->chordIsSet || !this->PhiIsSet || !this->alpha0IsSet){
        throw std::runtime_error("Module not fully setup yet");
    }
    this->Fij.resize(this->phi.size(), this->phi.size());
    for(size_t j = 0; j < this->phi.size(); j++){
        for(size_t i = 0; i < this->phi.size(); i++){
            double mode = static_cast<double>(j + 1);
            this->Fij(i, j) = ((4.0*this->span)/(this->lSlope[i]*this->chord[i]) + mode/(std::sin(this->phi[i])))*std::sin(mode*this->phi[i]);
        }
    }
    this->invFij = this->Fij.inverse();
    this->computeCoeffs();
    this->FijIsSet = true;
}

void PhillipsPhy::computeLift(){

    if(!this->PhiIsSet || !this->fCondsIsSet || !this->FijIsSet || !this->AIsSet || !this->BIsSet || !this->zOutIsSet){
        throw std::runtime_error("Requirements not met");
    }
    if(!this->isConnected){
        throw std::runtime_error("Module not connected/validated");
    }
    if(!(this->rLiftIsConnected || this->rTLiftIsConnected || this->rRMomentIsConnected)) return;
    if(this->phi.size() != this->zOut.size() - 2){
        throw std::runtime_error("Size mismatch");
    }
    double sum;
    std::vector<double> liftTemp;
    Eigen::VectorXd AplusB = this->A + this->B;
    liftTemp.reserve(this->zOut.size());
    liftTemp.push_back(0.0);
    for(size_t i = 0; i < this->phi.size(); i++){
        sum = 0.0;
        for(size_t j = 0; j < this->phi.size(); j++){
            double mode = static_cast<double>(j + 1);
            sum += AplusB(j)*(std::sin(mode*this->phi[i]));
        }
        liftTemp.push_back(2.0*this->fConds.getDensity()*this->fConds.getVelocity().magnitudeSquared()*this->span*sum);
    }
    liftTemp.push_back(0.0);
    if(this->rLiftIsConnected && this->realLift){
        this->realLift->set(this->zOut, liftTemp); // Is this copied? I can't think of a better way to account for the realLift output not being connected. In the init function, the 0 values at the edges are set, but maybe I could just set liftTemp to be at the realLift pointer even if it is not connected. Not sure
    }
    if(this->rTLiftIsConnected && this->realTotalLift){
        *this->realTotalLift = trapz(this->zOut, liftTemp);
    }
    if(this->rRMomentIsConnected && this->realRollingMoment){
        *this->realRollingMoment = trapzProduct(this->zOut, liftTemp, this->zOut);
    }

}

void PhillipsPhy::computeCoeffs(){ //no checks since its a private method and I am lazy
    this->computeA();
    this->computeB();
}

void PhillipsPhy::computeA(){
    Eigen::VectorXd aj = this->invFij.rowwise().sum();
    for (size_t i = 0; i < aj.size(); i++) {
        if (std::abs(aj(i)) < this->tolerance) {
            aj(i) = 0.0;
        }
    }
    this->A = aj.array() * (this->alpha0 - this->alpha0L).array();
    this->AIsSet = true;
}

void PhillipsPhy::computeB(){
    this->B = this->invFij*this->interpTwistEigen;
    this->BIsSet = true;
}

void PhillipsPhy::refreshZ(){
    if(!this->isConnected){
        throw std::runtime_error("Module connections not validated yet");
    }
    if(this->rTwistIsConnected && this->rGeometryIsConnected){
        this->readZTwist();
        this->readZGeo();
        if(!checkEdges(this->zTwist, this->zGeo, this->zOut, this->tolerance)){
            throw std::runtime_error("Mismatched edges");
        }
    } else if(this->rTwistIsConnected){
        this->readZTwist();
        if(!checkEdges(this->zTwist, this->zTwist, this->zOut, this->tolerance)){
            throw std::runtime_error("Mismatched edges");
        }
    } else if(this->rGeometryIsConnected){
        this->readZGeo();
        if(!checkEdges(this->zGeo, this->zGeo, this->zOut, this->tolerance)){
            throw std::runtime_error("Mismatched edges");
        }
    } else {
        throw std::runtime_error("Cannot read z vector. No input connected");
    }
    this->zIsSet = true;
}

void PhillipsPhy::refreshData(){

    if(!this->isConnected){
        throw std::runtime_error("Module connections not validated yet");
    }
    if(this->rTwistIsConnected && this->rGeometryIsConnected){
        this->readTwist();
        this->readGeo();
    } else if(this->rTwistIsConnected){
        this->readTwist();
    } else if(this->rGeometryIsConnected){
        this->readGeo();
    } else {
        throw std::runtime_error("Cannot read data vector. No input connected");
    }
    this->dataIsSet = true;

}

void PhillipsPhy::readZTwist(){

    if(!this->isConnected || !this->realTwist || !this->rTwistIsConnected){
        throw std::runtime_error("Invalid module connections");
    }
    if(!this->realTwist->hasNewCoords()){
        return;
    }
    if(this->realTwist->size() < 4){
        throw std::runtime_error("Vector too small");
    }
    for(size_t i = 1; i < this->realTwist->size(); i++){
        if(this->realTwist->z_at(i) < this->realTwist->z_at(i-1)){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->zTwist = this->realTwist->coords();
}

void PhillipsPhy::readZGeo(){

    if(!this->isConnected || !this->realGeometry || !this->rGeometryIsConnected){
        throw std::runtime_error("Invalid module connections");
    }
    if(this->realGeometry->size() < 4){
        throw std::runtime_error("Vector too small");
    }
    if(!this->realGeometry->hasNewCoords()){
        return;
    }
    for(size_t i = 1; i < this->realGeometry->size(); i++){
        if(this->realGeometry->z_at(i) < this->realGeometry->z_at(i-1)){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->zGeo = this->realGeometry->coords();
}

void PhillipsPhy::readTwist(){

    if(!this->isConnected || !this->realTwist || !this->rTwistIsConnected || !this->iTypeIsSet || !this->zIsSet){
        throw std::runtime_error("Invalid module connections");
    } // I would like more checks to be safe but these could be read every step so better keep it fast
    if(!this->realTwist->hasNewData()){
        return;
    }
    if(this->realTwist->size() != this->zTwist.size()){
        throw std::runtime_error("Size mismatch");
    }
    this->interpTwist = uasisi::interpolate(this->zTwist, this->realTwist->SWData(), this->zOut, getInterpType(this->iType));
    this->interpTwist.pop_back();
    this->interpTwist.erase(this->interpTwist.begin());
    this->interpTwistEigen = uasisi::vecAsEigen(this->interpTwist);
    if(this->isSet){
        this->computeB(); // This might be computed twice sometimes but I will take the sacrifice in exchange for better efficiency when run without airfoil input.
    }
    this->twistIsSet = true;
}

void PhillipsPhy::readGeo(){

    if(!this->isConnected || !this->realGeometry || !this->rGeometryIsConnected || !this->iTypeIsSet || !this->zIsSet){
        throw std::runtime_error("Invalid module connections");
    }
    if(!this->realGeometry->hasNewData()){
        return;
    }
    if(this->realGeometry->size() != this->zGeo.size()){
        throw std::runtime_error("Size mismatch");
    }
    this->interpGeo = uasisi::interpolate(this->zGeo, this->realGeometry->SWData(), this->zOut, getInterpType(this->iType));
    this->interpGeo.pop_back();
    this->interpGeo.erase(this->interpGeo.begin());
    this->chord = uasisi::extractMemberVector(this->interpGeo, &airfoil::getChord);
    this->chordIsSet = true;
    this->foilIsSet = true;
    this->computeRe();
    this->obtainPolars(); //Putting them here will make sure these functions are only run if there is newData. Good practice when writing uasisi modules will be to never reset the outputs when the data does not change.
    this->generateFij(); //Im puttin this here but Im not quite sure if I have made sure that all requirements are satisfied.
}

void PhillipsPhy::setZOut(const std::vector<double>& zNew){

    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(zNew.size() < 4 || zNew.size() % 2){ //Has to be even so after removing edge coordinates, an even number of internal points remain.
        throw std::runtime_error("Invalid vector. zOut must have an even number of points. Min. 4");
    }
    for(size_t i = 1; i < zNew.size(); i++){
        if(zNew[i] < zNew[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->span = zNew.back() - zNew.front();
    this->zOut = zNew;
    this->zOutIsSet = true;
    this->spanIsSet = true;
}

void PhillipsPhy::setAlpha0(const SpanwiseVec<double>& alphaNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->alpha0IsSet){
        throw std::runtime_error("alpha0 already set");
    }
    if(!this->zOutIsSet || !this->iTypeIsSet){
        throw std::runtime_error("zOut and iType must be set first");
    }
    if(alphaNew.size() < 4){
        throw std::runtime_error("Size error");
    }
    this->alpha0 = uasisi::vecAsEigen(uasisi::interpolate(alphaNew.coords(), alphaNew.SWData(), this->zOut, getInterpType(this->iType)));
    this->alpha0 = this->alpha0.segment(1, this->alpha0.size()-2);
    this->alpha0IsSet = true;
}

void PhillipsPhy::setChord(const SpanwiseVec<double>& chordNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->chordIsSet){
        throw std::runtime_error("Chord already set");
    }
    if(!this->zOutIsSet || !this->iTypeIsSet){
        throw std::runtime_error("zOut and iType must be set first");
    }
    if(chordNew.size() < 4){
        throw std::runtime_error("Size error");
    }
    this->chord = uasisi::interpolate(chordNew.coords(), chordNew.SWData(), this->zOut, getInterpType(this->iType));
    this->chord.pop_back();
    this->chord.erase(this->chord.begin());
    this->chordIsSet = true;
}

void PhillipsPhy::setFoil(const SpanwiseVec<airfoil>& foilNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->foilIsSet){
        throw std::runtime_error("Airfoil vector already set");
    }
    if(!this->zOutIsSet || !this->iTypeIsSet){
        throw std::runtime_error("zOut and iType must be set first");
    }
    if(foilNew.size() < 4){
        throw std::runtime_error("Size error");
    }
    this->interpGeo = uasisi::interpolate(foilNew.coords(), foilNew.SWData(), this->zOut, getInterpType(this->iType));
    this->interpGeo.pop_back();
    this->interpGeo.erase(this->interpGeo.begin());
    this->foilIsSet = true;
}

void PhillipsPhy::setTwist(const SpanwiseVec<double>& twistNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->twistIsSet){
        throw std::runtime_error("Twist vector already set");
    }
    if(!this->zOutIsSet || !this->iTypeIsSet){
        throw std::runtime_error("zOut and iType must be set first");
    }
    if(twistNew.size() < 4){
        throw std::runtime_error("Size error");
    }
    this->interpTwist = uasisi::interpolate(twistNew.coords(), twistNew.SWData(), this->zOut, getInterpType(this->iType));
    this->interpTwist.pop_back();
    this->interpTwist.erase(this->interpTwist.begin());
    this->twistIsSet = true;
}

void PhillipsPhy::updateFConds(const FlightConditions& fCondsNew){
    this->fConds = fCondsNew;
    this->fConds.comp();
    this->fCondsIsSet = true;
}

void PhillipsPhy::setIType(const interpType& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = tNew;
    this->iTypeIsSet = true;
}

void PhillipsPhy::setDefaultFoil(const std::string& nameNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->defaultFoil = nameNew;
}

void PhillipsPhy::setDefaultTwist(double twistNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->defaultTwist = twistNew;
}

void PhillipsPhy::setDefaultChord(double chordNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->defaultChord = chordNew;
}

void PhillipsPhy::setTolerance(double tolNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(tolNew < 0){
        throw std::runtime_error("Invalid or negative tolerance");
    }
    this->tolerance = tolNew;
}

void PhillipsPhy::setNPoints(size_t nNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->nPoints = nNew;
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


    
double trapzProduct(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z){

    if(x.size() != y.size() || x.size() != z.size() || x.size() < 4){
        throw std::runtime_error("vector size error");
    }
    double sum = 0.0;
    for(size_t i = 1; i < x.size(); i++){
        sum += 0.5 * (y[i]*z[i] + y[i-1]*z[i-1]) * (x[i] - x[i-1]);
    }
    return sum;

}

bool checkEdges(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z, double tol){
    if(x.size() < 2 || y.size() < 2 || z.size() < 2 || tol < 0){
        throw std::runtime_error("Input vectors are too small or negative tolerance");
    }
    bool checkStart = (std::abs(x[0] - y[0]) < tol) && (std::abs(x[0] - z[0]) < tol);
    bool checkEnd = (std::abs(x.back() - y.back()) < tol) && (std::abs(x.back() - z.back()) < tol);
    return checkStart && checkEnd;
}

}
namespace{
static uasisi::ModuleRegistration<uasisi::PhillipsPhy> registrationPhillipsPhy("phillipsPhy");
}
