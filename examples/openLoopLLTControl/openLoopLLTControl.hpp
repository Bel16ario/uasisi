#ifndef OPENLOOPCONTROL_HPP
#define OPENLOOPCONTROL_HPP

#include <asm-generic/ioctls.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <pybind11/stl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "control/constantCtrl/constantCtrl.hpp"
#include "uasisi/core/module.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "uasisi/core/types.hpp"

#include "optimizer/randomOpt/randomOpt.hpp"
#include "monitors/simpleLogger/simpleLogger.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "actuators/perfectAct/perfectAct.hpp"
#include "control/openLoopLLTCtrl/openLoopLLTCtrl.hpp"
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "uasisi/utils/pybindhelper.hpp"
using namespace uasisi;

#define NPTS 200
#define NACTS 74
#define SPAN 6.0
#define STPSZ 0.001
#define NTRIALS 30
    
inline double toRad(double theta){return (theta*M_PI)/180.0;}

inline std::vector<double> maxPos(NACTS + 2, toRad(5.0)); 
inline std::vector<double> maxVel(NACTS + 2, 0.5); 
inline std::vector<double> maxAccel(NACTS + 2, 0.2); 
inline std::vector<double> centerPos(NACTS + 2, toRad(0.0)); 
inline std::vector<double> theta(NACTS + 2, 0.0); 
inline std::vector<double> omega(NACTS + 2, 0.0); 

inline std::vector<double> zPts;
inline std::vector<double> zActs;

inline int getTermWidth(){
    struct winsize term;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &term);
    return term.ws_col;
}

inline void printProgress(std::ostream& console, double progress, int steps, int uRate){
    int width = getTermWidth();
    double progressPC = progress*100;
    int updates;
    if(!uRate){
        updates = 0;
    } else {
        updates = static_cast<int>(steps / uRate);
    }
    std::ostringstream info;
    info << " " << std::fixed << std::setprecision(1) << progressPC << "% | ";
    info << steps << " steps";
    if(uRate > 0){
    info << " | " << updates << " param updates";
    }
    std::string infoStr = info.str();
    int infolen = infoStr.length();
    if(infolen > width){
        return;
    }
    width -= infolen;
    if(!(width < 3)){
        int filled = static_cast<int>(progress*(width - 2));
        console << "\r[";
        for(size_t i = 0; i < width - 2; i++){
            if(i < filled){
                console << "█";
            } else {
                console << " ";
            }
        }
        console << "]" << infoStr << std::flush;
    } else {
        console << "\r" << infoStr << std::flush;
    }

}

inline size_t nSteps;

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
    phyPtr->setNSWPoints(NPTS);
    phyPtr->updateFConds(fConds);
}

inline void LLTControl(IModule* mod){
    OpenLoopLLTCtrl* ctrlPtr = dynamic_cast<OpenLoopLLTCtrl*>(mod);
    if(!ctrlPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    std::vector<double> chordsCtrl(NPTS + 2, 1.0); 
    std::vector<double> slopeCtrl(NPTS + 2, 6.5317); 
    std::vector<double> angleCtrl(NPTS + 2, -0.0375); 
    std::vector<double> maxPosCtrl(NPTS + 2, toRad(5)); 
    std::vector<double> centerPosCtrl(NPTS + 2, 0); 
    FlightConditions fConds;
    fConds.setVInf(point{166.0, 0.0, 0.0, 0});
    fConds.setAltitude(1000.0);
    fConds.comp();

    ctrlPtr->setIType(interpType::LIN);

    ctrlPtr->setZ(zPts);
    ctrlPtr->setActCoords(zActs);
    ctrlPtr->setNSWPoints(NPTS);
    ctrlPtr->setChords(chordsCtrl);
    ctrlPtr->setClSlope(slopeCtrl);
    ctrlPtr->setRho(fConds.getDensity());
    ctrlPtr->setSpan(SPAN);
    ctrlPtr->setThetaCenter(centerPosCtrl);
    ctrlPtr->setThetaMax(maxPosCtrl);
    ctrlPtr->setZeroLiftAngle(angleCtrl);
    ctrlPtr->setVInf(fConds.getVelocity().magnitude());
}

inline void constantAccelActConfig(IModule* mod){
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

inline void constantAccelActConfig2(IModule* mod){
    ConstantAccelAct* actPtr = dynamic_cast<ConstantAccelAct*>(mod);
    if(!actPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    
    std::vector<double> thetaStart(NACTS + 2, 0); 

    actPtr->setIType(interpType::LIN);
    
    actPtr->setZ(zActs);
    actPtr->setZOut(zPts);
    actPtr->setMaxPos(maxPos);
    actPtr->setMaxVel(maxVel);
    actPtr->setMaxAccel(maxAccel);
    actPtr->setCenterPos(centerPos);
    actPtr->setTheta(thetaStart);
    actPtr->setOmega(omega);
    actPtr->setThickness(0.05);
    actPtr->setAddThickness(true);

}

inline void simpleLogger(IModule* mod){
    SimpleLogger* monPtr = dynamic_cast<SimpleLogger*>(mod);
    if(!monPtr){
        throw std::runtime_error("Problem casting from IModule class");
    }
    monPtr->setWriteInterval(100);
}

#endif
