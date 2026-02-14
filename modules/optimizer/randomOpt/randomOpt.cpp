#include "randomOpt.hpp"
#include "uasisi/core/types.hpp"
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <random>
#include <boost/math/distributions/normal.hpp>

namespace uasisi{

RandomOpt::RandomOpt(){

    std::cout << "Random Optimizer created \n";

}

void RandomOpt::init(){ // 
    
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->zIsSet || !this->maskIsSet || !this->pointRangeIsSet || !this->zOutIsSet || !this->iTypeIsSet || !this->updatePeriodIsSet){
        throw std::runtime_error("Error, Aribitrary Optimizer Module not fully setup");
    }
    if(this->z.size() != this->upperMask.size() || this->z.size() != this->lowerMask.size()){
        throw std::runtime_error("Size mismatch");
    } //Z and ZOut edges should be checked. I need to put the edge checking function in phillipp's module somewhere commonly accessible. TODO
    if(!this->isConnected){
        throw std::runtime_error("Module not connected yet");
    }

    if(this->tLiftConnected){
        this->generateDistribution();
    }
    
    this->isSet = true;

}

std::vector<SignalInfo> RandomOpt::declareSignals(){
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(1);
    reqSignals.push_back(SignalInfo("targetLift", DataType::DOB, SignalType::OUT));

    return reqSignals;
}

void RandomOpt::validateConnections(){
    if(!targetLift){
        std::cout << "WARNING: output unconnected\n";
        this->tLiftConnected = false;
    } else {
        this->tLiftConnected = true;
    }
    this->isConnected = true;
}

void RandomOpt::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){
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

void RandomOpt::step(double t, double dt){
    this->timeSinceUpdate += dt;
    if(this->timeSinceUpdate >= this->updatePeriod){
        this->generateDistribution();
        this->timeSinceUpdate = 0.0;
    }
}

void RandomOpt::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    for(size_t i = 1; i < zNew.size(); i++){
        if(zNew[i] < zNew[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->z = zNew;
    this->minZ = zNew[0];
    this->maxZ = zNew.back();
    this->span = this->maxZ - this->minZ;
    this->zIsSet = true;
}

void RandomOpt::setZOut(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    for(size_t i = 1; i < zNew.size(); i++){
        if(zNew[i] < zNew[i-1]){
            throw std::runtime_error("Points in vector are not ordered");
        }
    }
    this->zOut = zNew;
    this->zOutIsSet = true;
}

void RandomOpt::setMask(const std::vector<double>& mNewMin, const std::vector<double>& mNewMax){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->zIsSet){
        throw std::runtime_error("Z needs to be set");
    }
    if(mNewMin.size() != mNewMax.size() || mNewMin.size() != this->z.size()){
        throw std::runtime_error("Size mismatch");
    }
    this->upperMask = mNewMax;
    this->lowerMask = mNewMin;
    this->maskIsSet = true;
}

void RandomOpt::setPointRange(size_t nNewMin, size_t nNewMax){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nNewMin >= nNewMax || nNewMin < 3){
        throw std::runtime_error("Invalid amount of points");
    }
    this->maxPoints = nNewMax;
    this->minPoints = nNewMin;
    this->pointRangeIsSet = true;
}

void RandomOpt::setUpdatePeriod(double tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(tNew < 0.0){
        throw std::runtime_error("Invalid time"); //Later could be cool to add -1 for unlimited time, 0 for on every step, etc.
    }
    this->updatePeriod = tNew;
    this->updatePeriodIsSet = true;
}

void RandomOpt::setRelativeTolerance(double tolNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(tolNew < 0.0){
        throw std::runtime_error("Invalid tolerance"); 
    }
    this->relativeTolerance = tolNew;
}

void RandomOpt::setBaselineSigma(double sigmaNew){ //Can be called when running
    if(sigmaNew < 0.0){
        throw std::runtime_error("Invalid standard deviation"); 
    }
    this->baselineSigma = sigmaNew;
}

void RandomOpt::setIType(const interpType& t){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = t;
    this->iTypeIsSet = true;
}

void RandomOpt::generateDistribution(){
    if(!this->zIsSet || !this->maskIsSet || !this->zOutIsSet || !this->iTypeIsSet || !this->pointRangeIsSet){
        throw std::runtime_error("Module not fully configured");
    }
    size_t nPts = randomNumberFrom(this->minPoints, this->maxPoints);
    bool startFromLeft = randomNumberFrom(0, 1);
    std::vector<double> zNew;
    std::vector<double> lNew;
    std::vector<size_t> idx;
    zNew.reserve(nPts);
    lNew.reserve(nPts);
    idx.resize(nPts);
    zNew.push_back(this->minZ);
    lNew.push_back(randomDoubleFrom(-1.0, 1.0));
    std::iota(idx.begin(), idx.end(), 0);
    double zNewPt;
    bool breakFlag = false;
    this->tolerance = this->span*this->relativeTolerance;
    this->tolerance = std::min(this->tolerance, (0.5*this->span)/(nPts - 1));
    for(size_t i = 0; i < nPts - 2; i++){
        breakFlag = false;
        zNewPt = randomDoubleFrom(this->minZ, this->maxZ);
        for(double zPt : zNew){
            if((zNewPt >= zPt - this->tolerance) && (zNewPt <= zPt + this->tolerance)){
                i--;
                breakFlag = true;
                break;
            }
        }
        if(breakFlag) continue;
        zNew.push_back(zNewPt);
        lNew.push_back(0.0);
    }
    zNew.push_back(this->maxZ);
    lNew.push_back(randomDoubleFrom(-1, 1));
    std::sort(idx.begin(), idx.end(), [&zNew](size_t a, size_t b) {
        return zNew[a] < zNew[b];
    });
    zNew = reorderFrom(zNew, idx);
    lNew = reorderFrom(lNew, idx);
    std::fill(idx.begin(), idx.end(), 0);
    idx[0] = 1;
    idx[nPts - 1] = 1;
    size_t lPoint;
    size_t rPoint;
    double minR;
    double sigma;
    double mu;
    if(startFromLeft){
        for(size_t i = 1; ; i++){
            if(idx[i]) break;
            for(size_t j = i-1; ; j--){
                if(idx[j]) {lPoint = j; break;}
            }
            for(size_t j = i+1;; j++){
                if(idx[j]) {rPoint = j; break;}
            }
            mu = linearInterp(zNew[lPoint], lNew[lPoint], zNew[rPoint], lNew[rPoint], zNew[i]);
            if(zNew[i] - zNew[lPoint] > zNew[rPoint] - zNew[i]){
                minR = zNew[rPoint] - zNew[i];
            } else {
                minR = zNew[i] - zNew[lPoint];
            }
            sigma = (minR/this->span)*this->baselineSigma;
            lNew[i] = sampleFromTruncatedNormal(mu, sigma, -1, 1);
            idx[i] = 1;
            if(idx[nPts - 1 - i]) break;
            for(size_t j = nPts-2-i; ; j--){
                if(idx[j]) {lPoint = j; break;}
            }
            for(size_t j = nPts - i; ; j++){
                if(idx[j]) {rPoint = j; break;}
            }
            mu = linearInterp(zNew[lPoint], lNew[lPoint], zNew[rPoint], lNew[rPoint], zNew[nPts - 1 - i]);
            if(zNew[nPts - 1 - i] - zNew[lPoint] > zNew[rPoint] - zNew[nPts - 1 - i]){
                minR = zNew[rPoint] - zNew[nPts - 1 - i];
            } else {
                minR = zNew[nPts - 1 - i] - zNew[lPoint];
            }
            sigma = (minR/this->span)*this->baselineSigma;
            lNew[nPts - 1 - i] = sampleFromTruncatedNormal(mu, sigma, -1, 1);
            idx[nPts - 1 - i] = 1;
        }
    } else {
        for(size_t i = 1; ; i++){
            if(idx[nPts - 1 - i]) break;
            for(size_t j = nPts-2-i; ; j--){
                if(idx[j]) {lPoint = j; break;}
            }
            for(size_t j = nPts - i; ; j++){
                if(idx[j]) {rPoint = j; break;}
            }
            mu = linearInterp(zNew[lPoint], lNew[lPoint], zNew[rPoint], lNew[rPoint], zNew[nPts - 1 - i]);
            if(zNew[nPts - 1 - i] - zNew[lPoint] > zNew[rPoint] - zNew[nPts - 1 - i]){
                minR = zNew[rPoint] - zNew[nPts - 1 - i];
            } else {
                minR = zNew[nPts - 1 - i] - zNew[lPoint];
            }
            sigma = (minR/this->span)*this->baselineSigma;
            lNew[nPts - 1 - i] = sampleFromTruncatedNormal(mu, sigma, -1, 1);
            idx[nPts - 1 - i] = 1;
            if(idx[i]) break;
            for(size_t j = i-1; ; j--){
                if(idx[j]) {lPoint = j; break;}
            }
            for(size_t j = i+1;; j++){
                if(idx[j]) {rPoint = j; break;}
            }
            mu = linearInterp(zNew[lPoint], lNew[lPoint], zNew[rPoint], lNew[rPoint], zNew[i]);
            if(zNew[i] - zNew[lPoint] > zNew[rPoint] - zNew[i]){
                minR = zNew[rPoint] - zNew[i];
            } else {
                minR = zNew[i] - zNew[lPoint];
            }
            sigma = (minR/this->span)*this->baselineSigma;
            lNew[i] = sampleFromTruncatedNormal(mu, sigma, -1, 1);
            idx[i] = 1;
        }
    }

    std::vector<double> lowerMaskZ;
    std::vector<double> upperMaskZ;
    lowerMaskZ.reserve(nPts);
    upperMaskZ.reserve(nPts);
    lowerMaskZ = interpolate(this->z, this->lowerMask, zNew, getInterpType(this->iType));
    upperMaskZ = interpolate(this->z, this->upperMask, zNew, getInterpType(this->iType));
    for(size_t i = 0; i < nPts; i++){
        lNew[i] *= 0.5*(upperMaskZ[i] - lowerMaskZ[i]);
        lNew[i] += 0.5*(upperMaskZ[i] - lowerMaskZ[i]);
    }
    this->targetLift->set(this->zOut, uasisi::interpolate(zNew, lNew, this->zOut, getInterpType(this->iType)));
}

size_t randomNumberFrom(size_t min, size_t max){
    static std::mt19937_64 rng{std::random_device{}()};
    std::uniform_int_distribution<size_t> dist(min, max);
    return dist(rng);
}

double randomDoubleFrom(double min, double max){
    static std::mt19937_64 rng{std::random_device{}()};
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

std::vector<double> reorderFrom(const std::vector<double>& data, const std::vector<size_t>& order){
    if(data.size() != order.size()){
        throw std::runtime_error("Size mismatch");
    }
    if(data.empty()){
        throw std::runtime_error("Empty vector");
    }
    std::vector<double> result;
    result.resize(order.size());
    for(size_t i = 0; i < order.size(); i++){
        result[i] = data[order[i]];
    }
    return result;
}

double linearInterp(double z0, double f0, double z1, double f1, double z){
    double m = (f1 - f0)/(z1 - z0);
    return (f0 + m*(z-z0));
}

double sampleFromTruncatedNormal(double mean, double stddev, double min, double max){
    if(stddev < 1e-12){
        return std::clamp(mean, min, max);
    }
    static std::mt19937_64 rng{std::random_device{}()};
    boost::math::normal_distribution<> dist(mean, stddev);
    double a = boost::math::cdf(dist, min);
    double b = boost::math::cdf(dist, max);
    std::uniform_real_distribution<double> uniform(a, b);
    double u = uniform(rng);
    return boost::math::quantile(dist, u);
}

}

namespace{
static uasisi::ModuleRegistration<uasisi::RandomOpt> registrationRandomOpt("randomOpt");
}
