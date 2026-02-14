#ifndef REINFORCEMENTLEARNING_HPP
#define REINFORCEMENTLEARNING_HPP

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <pybind11/stl.h>
#include "control/constantCtrl/constantCtrl.hpp"
#include "uasisi/core/module.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "uasisi/core/types.hpp"

#include "optimizer/randomOpt/randomOpt.hpp"
#include "monitors/simpleLogger/simpleLogger.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "modules/pythonWrapper/pythonWrapper.hpp"
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "uasisi/utils/pybindhelper.hpp"
using namespace uasisi;

#define NPTS 200
#define NACTS 16
#define SPAN 6.0
#define STPSZ 0.001
#define UPDR 400
#define NUPDSTRAIN 1000
#define NUPDSTEST 30
    
inline double toRad(double theta){return (theta*M_PI)/180.0;}

inline std::vector<double> maxPos(NACTS + 2, toRad(5.0)); 
inline std::vector<double> maxVel(NACTS + 2, 0.5); 
inline std::vector<double> maxAccel(NACTS + 2, 0.2); 
inline std::vector<double> centerPos(NACTS + 2, toRad(0.0)); 
inline std::vector<double> theta(NACTS + 2, 0.0); 
inline std::vector<double> omega(NACTS + 2, 0.0); 

inline std::vector<double> zPts;
inline std::vector<double> zActs;

size_t nSteps;

inline void constantCtrlConfig1(IModule* mod){
    ConstantCtrl* ctrlPtr = dynamic_cast<ConstantCtrl*>(mod);
    if(!ctrlPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    std::vector<double> ctrlCoords = {-SPAN/2, SPAN/2};
    std::vector<double> ctrlData = {toRad(5.0), toRad(5.0)};

    ctrlPtr->setIType(interpType::LIN);
    ctrlPtr->setDType(DataType::DOB);

    ctrlPtr->setZ(ctrlCoords);
    ctrlPtr->setSData(ctrlData);
    ctrlPtr->setActCoords(zActs);
}

inline void constantAccelActConfig1(IModule* mod){
    ConstantAccelAct* actPtr = dynamic_cast<ConstantAccelAct*>(mod);
    if(!actPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    
    std::vector<double> thetaLow(NACTS + 2, toRad(-5)); 

    actPtr->setIType(interpType::LIN);
    
    actPtr->setZ(zActs);
    actPtr->setZOut(zPts);
    actPtr->setMaxPos(maxPos);
    actPtr->setMaxVel(maxVel);
    actPtr->setMaxAccel(maxAccel);
    actPtr->setCenterPos(centerPos);
    actPtr->setTheta(thetaLow);
    actPtr->setOmega(omega);
    actPtr->setThickness(0.05);
    actPtr->setAddThickness(true);

}

inline void phillipsPhyConfig1(IModule* mod){
    PhillipsPhy* phyPtr = dynamic_cast<PhillipsPhy*>(mod);
    if(!phyPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    FlightConditions fConds;
    fConds.setVInf(point{166.0, 0.0, 0.0, 0});
    fConds.setAltitude(1000.0);
    
    phyPtr->setIType(interpType::LIN);

    phyPtr->setZOut(zPts);
    phyPtr->setNSWPoints(200);
    phyPtr->updateFConds(fConds);
}

inline void constantAccelActConfig2(IModule* mod){
    ConstantAccelAct* actPtr = dynamic_cast<ConstantAccelAct*>(mod);
    if(!actPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    actPtr->setIType(interpType::LIN);
    
    actPtr->setZ(zActs);
    actPtr->setZOut(zPts);
    actPtr->setMaxPos(maxPos);
    actPtr->setMaxVel(maxVel);
    actPtr->setMaxAccel(maxAccel);
    actPtr->setCenterPos(centerPos);
    actPtr->setTheta(theta);
    actPtr->setOmega(omega);
    actPtr->setThickness(0.05);
    actPtr->setAddThickness(true);

}

inline void pythonWrapperConfig2(IModule* mod){
    PythonWrapper* pyPtr = dynamic_cast<PythonWrapper*>(mod);
    if(!pyPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    pyPtr->setScriptPath("../examples/reinforcementLearning/reinforcementLearning.py");
    pyPtr->loadScript();
    PythonConfigDict emptyDict;
    pyPtr->execCommand("uasisiConfig.updatePeriod = " + std::to_string(nSteps*2), emptyDict);
    pyPtr->execCommand("uasisiConfig.training = True", emptyDict);

    py::module_ torchMod = py::module_::import("torch");
    py::object pyActCoords = torchMod.attr("Tensor")(py::cast(zActs));
    py::object pyMaxPos = torchMod.attr("Tensor")(py::cast(maxPos));
    py::object pyCenterPos = torchMod.attr("Tensor")(py::cast(centerPos));
    py::object pyTheta = torchMod.attr("Tensor")(py::cast(theta));
    PythonConfigDict actCoordsConfig;
    PythonConfigDict maxPosConfig;
    PythonConfigDict centerPosConfig;
    PythonConfigDict thetaConfig;
    actCoordsConfig.setConfigObject("actCoords", pyActCoords);
    maxPosConfig.setConfigObject("maxPos", pyMaxPos);
    centerPosConfig.setConfigObject("centerPos", pyCenterPos);
    thetaConfig.setConfigObject("theta", pyTheta);
    pyPtr->execCommand("uasisiConfig.actCoords = actCoords", actCoordsConfig);
    pyPtr->execCommand("uasisiConfig.maxPos = maxPos", maxPosConfig);
    pyPtr->execCommand("uasisiConfig.centerPos = centerPos", centerPosConfig);
    pyPtr->execCommand("uasisiConfig.theta = theta", thetaConfig);
}

inline void phillipsPhyConfig2(IModule* mod){
    PhillipsPhy* phyPtr = dynamic_cast<PhillipsPhy*>(mod);
    if(!phyPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }

    FlightConditions fConds;
    fConds.setVInf(point{166.0, 0.0, 0.0, 0});
    fConds.setAltitude(1000.0);
    
    phyPtr->setIType(interpType::LIN);

    phyPtr->setZOut(zPts);
    phyPtr->setNSWPoints(200);
    phyPtr->updateFConds(fConds);
}

inline void simpleLogger2(IModule* mod){
    SimpleLogger* monPtr = dynamic_cast<SimpleLogger*>(mod);
    if(!monPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    monPtr->setWriteInterval(100);
}

inline void pythonWrapperConfig3(IModule* mod){
    PythonWrapper* pyPtr = dynamic_cast<PythonWrapper*>(mod);
    if(!pyPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    pyPtr->setScriptPath("../examples/reinforcementLearning/reinforcementLearning.py");
    pyPtr->loadScript();
    PythonConfigDict emptyDict;
    pyPtr->execCommand("uasisiConfig.updatePeriod = " + std::to_string(nSteps*2), emptyDict);
    pyPtr->execCommand("uasisiConfig.training = False", emptyDict);
    
    py::module_ torchMod = py::module_::import("torch");
    py::object pyActCoords = torchMod.attr("Tensor")(py::cast(zActs));
    py::object pyMaxPos = torchMod.attr("Tensor")(py::cast(maxPos));
    py::object pyCenterPos = torchMod.attr("Tensor")(py::cast(centerPos));
    py::object pyTheta = torchMod.attr("Tensor")(py::cast(theta));
    PythonConfigDict actCoordsConfig;
    PythonConfigDict maxPosConfig;
    PythonConfigDict centerPosConfig;
    PythonConfigDict thetaConfig;
    actCoordsConfig.setConfigObject("actCoords", pyActCoords);
    maxPosConfig.setConfigObject("maxPos", pyMaxPos);
    centerPosConfig.setConfigObject("centerPos", pyCenterPos);
    thetaConfig.setConfigObject("theta", pyTheta);
    pyPtr->execCommand("uasisiConfig.actCoords = actCoords", actCoordsConfig);
    pyPtr->execCommand("uasisiConfig.maxPos = maxPos", maxPosConfig);
    pyPtr->execCommand("uasisiConfig.centerPos = centerPos", centerPosConfig);
    pyPtr->execCommand("uasisiConfig.theta = theta", thetaConfig);

}

#endif
