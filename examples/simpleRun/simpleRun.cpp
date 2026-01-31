#include <iostream>
#include <stdexcept>
#include <vector>
#include "uasisi/core/module.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "uasisi/core/types.hpp"

#include "monitors/simpleLogger/simpleLogger.hpp"
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "control/constantCtrl/constantCtrl.hpp"

using namespace uasisi;

void constantCtrlConfig(IModule* mod){
    ConstantCtrl* ctrlPtr = dynamic_cast<ConstantCtrl*>(mod);
    if(!ctrlPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    std::vector<double> ctrlCoords = {-5.0, 5.0};
    std::vector<double> ctrlData = {-5.0, 5.0};
    std::vector<double> actCoords = {-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0};

    ctrlPtr->setIType(interpType::LIN);
    ctrlPtr->setDType(DataType::DOB);

    ctrlPtr->setZ(ctrlCoords);
    ctrlPtr->setSData(ctrlData);
    ctrlPtr->setActCoords(actCoords);
}

void constantAccelActConfig(IModule* mod){
    ConstantAccelAct* actPtr = dynamic_cast<ConstantAccelAct*>(mod);
    if(!actPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    std::vector<double> actCoords = {-5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> maxPos = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    std::vector<double> maxVel = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> maxAccel = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    std::vector<double> centerPos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> theta = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> omega = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> zOut(200);
    double dz = 10.0/199.0;
    for(size_t i = 0; i < zOut.size(); i++){
        zOut[i] = -5 + dz*i;
    }

    actPtr->setIType(interpType::CSP);
    
    actPtr->setZ(actCoords);
    actPtr->setZOut(zOut);
    actPtr->setMaxPos(maxPos);
    actPtr->setMaxVel(maxVel);
    actPtr->setMaxAccel(maxAccel);
    actPtr->setCenterPos(centerPos);
    actPtr->setTheta(theta);
    actPtr->setOmega(omega);
    actPtr->setThickness(0.05);
    actPtr->setAddThickness(true);

}

void phillipsPhyConfig(IModule* mod){
    PhillipsPhy* phyPtr = dynamic_cast<PhillipsPhy*>(mod);
    if(!phyPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    std::vector<double> zOut(200);
    double dz = 10.0/199.0;
    for(size_t i = 0; i < zOut.size(); i++){
        zOut[i] = -5 + dz*i;
    }
    FlightConditions fConds;
    fConds.setVInf(point{100.0, 0.0, 0.0, 0});
    fConds.setAltitude(1000.0);
    
    phyPtr->setIType(interpType::CSP);

    phyPtr->setZOut(zOut);
    phyPtr->updateFConds(fConds);
}


int main(int argc, char* argv[]){

    std::string fPath = "../results/simpleRunResults.h5";
    
    if(argc != 1){
        if(argc != 3 || std::string(argv[1]) != "-o"){
            throw std::runtime_error("Invalid arguments");
        }
        fPath = std::string(argv[2]);
    }

    std::cout << "Simple UASISI simulation run\n";

    Orchestrator orch;

    orch.createModule("constantCtrl", "ctrl", 0, constantCtrlConfig);
    orch.createModule("constantAccelAct", "act", 1, constantAccelActConfig);
    orch.createModule("phillipsPhy", "phy", 2, phillipsPhyConfig);
    orch.createModule("simpleLogger", "mon", 3, [&fPath](IModule* mod){
        SimpleLogger* monPtr = dynamic_cast<SimpleLogger*>(mod);
        if(!monPtr){
            throw std::runtime_error("Problem casting from IModule class");
        }
        monPtr->setFileName(fPath);
    });

    orch.configureModules();
    orch.connectSystem();
    orch.init();

    double dt = 0.01;
    double tMax = 1.0;
    double t = 0;

    for(size_t i = 0; t < tMax; i++){
        t = i*dt;
        orch.step(t, dt);
    }


    return 0;
}
