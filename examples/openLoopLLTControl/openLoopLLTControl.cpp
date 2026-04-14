#include <cmath>
#include <iomanip>
#include <ios>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include "uasisi/core/module.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "uasisi/core/types.hpp"
#include "openLoopLLTControl.hpp"
#include "optimizer/randomOpt/randomOpt.hpp"
#include "monitors/simpleLogger/simpleLogger.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "actuators/perfectAct/perfectAct.hpp"
#include "modules/pythonWrapper/pythonWrapper.hpp"
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "uasisi/utils/pybindhelper.hpp"

using namespace uasisi;

int main(int argc, char* argv[]){

    std::ostream nout(std::cout.rdbuf());
    auto* coutOld = std::cout.rdbuf();
    std::ofstream devNull("/dev/null");
    std::cout.rdbuf(devNull.rdbuf());

    nout << "====UASISI LLT DEMO====\n\n";
    nout << "Preparing preliminary run...";

    zPts.reserve(NPTS + 2);
    zPts.push_back(-SPAN/2);
    double dz = SPAN/(NPTS + 1);
    for(size_t i = 1; i < NPTS + 1; i++){
        zPts.push_back(-SPAN/2 + i*dz);
    }
    zPts.push_back(SPAN/2);

    zActs.reserve(NACTS + 2);
    zActs.push_back(-SPAN/2);
    dz = SPAN/(NACTS + 1);
    for(size_t i = 1; i < NACTS + 1; i++){
        zActs.push_back(-SPAN/2 + i*dz);
    }
    zActs.push_back(SPAN/2);

    //----------------------------------------FIRST RUN: OBTAIN LIFT MASKS AND ACTUATOR SETTLE TIME
    Orchestrator orch1;
    orch1.createModule("constantCtrl", "ctrl1", 0, constantCtrlConfig1);
    orch1.createModule("constantAccelAct", "act1", 1, constantAccelActConfig1);
    orch1.createModule("phillipsPhy", "phy1", 2, phillipsPhyConfig1);
    orch1.createModule("simpleLogger", "mon1", 4, simpleLogger);
    SimpleLogger* monPtr = dynamic_cast<SimpleLogger*>(orch1.getModule("mon1"));
    monPtr->setFileName("../results/preliminarRun.h5");
    orch1.configureModules();
    orch1.connectSystem();
    orch1.init();

    const SpanwiseVec<double>* realGeometry1 = orch1.getDOB("realGeometry");
    const SpanwiseVec<double>* realLift1 = orch1.getDOB("realLift");

    std::vector<double> lowerMask = realLift1->SWData();
    std::vector<double> pastState = realGeometry1->SWData();

    double t = 0.0;
    bool isFinished;
    size_t counter = 20;
    nSteps = 0;
    nout << "DONE\n";
    nout << "Running first simulation...";

    do{
        t+=STPSZ;
        orch1.step(t, STPSZ);
        nSteps++;
        isFinished = true;
        for(size_t i = 0; i < realGeometry1->size(); i++){
            if((*realGeometry1)[i] != pastState[i]){
                isFinished = false;
            }
        }
        std::cout << "\n";
        pastState = realGeometry1->SWData();
        if(counter){ //Skip checking on first steps
            counter--;
            isFinished = false;
        }

    } while(!isFinished);

    double tsettle = t;
    std::vector<double> upperMask = realLift1->SWData();
    std::vector<double> liftRange;
    liftRange.reserve(lowerMask.size());
    for(size_t i = 0; i < lowerMask.size(); i++){
        if(upperMask[i] - lowerMask[i] == 0){
            liftRange.push_back(1e-8);
        } else {
            liftRange.push_back(upperMask[i] - lowerMask[i]);
        }
    }
    nout << "DONE\n";
    nout << "Maximum actuator settling time:\t" << std::fixed << std::setprecision(3) << tsettle;
    nout << "s (" << std::to_string(nSteps) << " steps)\n";

    //---------------------------------------------SECOND RUN: TRAIN MODEL
    nout << "Preparing simulation run...\n";
    std::string fPath = "../results/LLTResults.h5";
    Orchestrator orch2;
    orch2.createModule("randomOpt", "opt", 0, [&lowerMask, &upperMask, &tsettle](IModule* mod){
        RandomOpt* optPtr = dynamic_cast<RandomOpt*>(mod);
        if(!optPtr){
            throw std::runtime_error("Problem casting from IModule class");
        }

        optPtr->setZ(zPts);
        optPtr->setZOut(zPts);
        optPtr->setMask(lowerMask, upperMask);
        optPtr->setPointRange(size_t(NACTS/2), NACTS*2);
        optPtr->setUpdatePeriod(tsettle);
        optPtr->setBaselineSigma(2.0);
        optPtr->setIType(interpType::CSP);
    });
    orch2.createModule("openLoopLLTCtrl", "ctrl2", 1, LLTControl);
    orch2.createModule("constantAccelAct", "act2", 2, constantAccelActConfig2);
    orch2.createModule("phillipsPhy", "phy2", 3, phillipsPhyConfig1);
    orch2.createModule("simpleLogger", "mon2", 4, simpleLogger);
    orch2.configureModules();
    monPtr = dynamic_cast<SimpleLogger*>(orch2.getModule("mon2"));

    monPtr->setFileName(fPath);
    
    orch2.connectSystem();
    orch2.init();
    t = 0.0;
    
    nout << "DONE\n";
    nout << "Running second simulation...\n";

    for(size_t i = 0; i < nSteps*NTRIALS; i++){
        t+=STPSZ;
        orch2.step(t, STPSZ);
        printProgress(nout, i/(static_cast<double>(nSteps*NTRIALS)), i+1, 0);
    }
    nout << "\n";
    nout << "Run finished \n";
    std::cout.rdbuf(coutOld);
}
