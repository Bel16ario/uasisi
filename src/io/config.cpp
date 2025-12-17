#include "uasisi/io/config.hpp"
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace uasisi{

Config::Config(const std::string& filename): root(YAML::LoadFile(filename)){
    std::cout << "Parsing " + filename + " for configuration" << std::endl;
    setWingSpan();
    if(this->wingSpan <= 0){
        throw std::runtime_error("Invalid wingspan");
    }
    setNActuators();
    if(this->nActuators <= 1){
        throw std::runtime_error("Invalid number of actuators (A minimum of 2 is required)");
    }
    setActLinSpaced();
    if(this->actLinSpaced){
        int i;
        double dx = 2.0 / (this->nActuators - 1);
        for(i = 0; i < this->nActuators; i++){
            this->actCoords.push_back(-1 + i*dx);
        }
    } else {
        setActCoords();
        if(this->actCoords.size() != this->nActuators){
            throw std::runtime_error("Size of coordinate vector must match the number of actuators!");
        }
        int i;
        for(i = 0; i < this->nActuators; i++){
            if(this->actCoords[i] < -1 || this->actCoords[i] > 1){
                throw std::runtime_error("Coodinate " + std::to_string(i) + "out of bounds (-1 to 1)");
            }
        }
    }
    setMWType();
    if(this->mWType == MWType::AOA || this->mWType == MWType::BOTH) setOmega();
    if(this->mWType == MWType::CAMBER || this->mWType == MWType::BOTH) setDelta();
    setVInf();
    setRho0();
    if(this->rho0 <= 0){
        throw std::runtime_error("Invalid density");
    }
    setMu0();
    if(this->mu0 <= 0){
        throw std::runtime_error("Invalid viscocity");
    }
    setAlpha();
    if(this->alpha <= 0){
        throw std::runtime_error("Invalid angle of attack");
    }
    setDt();
    if(this->dt <= 0){
        throw std::runtime_error("Invalid time step");
    }
    setTMax();
    if(this->tMax <= 0){
        throw std::runtime_error("Invalid simulation time");
    }
    double ratio = this->tMax / this->dt;
    if(this->dt > this->tMax || std::abs(ratio - std::round(ratio)) > 1e-9){
        throw std::runtime_error("Incompatible time step and simulation time");
    }

}

Config::~Config(){
    std::cout << "Config terminated" << std::endl;
}

}
