#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "control/constantCtrl/constantCtrl.hpp"
#include "uasisi/core/module.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "uasisi/core/types.hpp"

#include "reinforcementLearning.hpp"

#include "optimizer/randomOpt/randomOpt.hpp"
#include "monitors/simpleLogger/simpleLogger.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "modules/pythonWrapper/pythonWrapper.hpp"
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "uasisi/utils/pybindhelper.hpp"

using namespace uasisi;

int main(int argc, char* argv[]){
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
    orch1.createModule("simpleLogger", "mon1", 4, simpleLogger2);
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

    //---------------------------------------------SECOND RUN: TRAIN MODEL
    std::string fPath = "../results/reinforcementLearningTRAIN.h5";
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
        optPtr->setUpdatePeriod(tsettle*4);
        optPtr->setIType(interpType::CSP);
    });
    orch2.createModule("pythonWrapper", "ctrl2", 1, pythonWrapperConfig2);
    orch2.createModule("constantAccelAct", "act2", 2, constantAccelActConfig2);
    orch2.createModule("phillipsPhy", "phy2", 3, phillipsPhyConfig2);
    orch2.createModule("simpleLogger", "mon2", 4, simpleLogger2);
    orch2.configureModules();
    monPtr = dynamic_cast<SimpleLogger*>(orch2.getModule("mon2"));
    PythonWrapper* ctrlPtr = dynamic_cast<PythonWrapper*>(orch2.getModule("ctrl2"));

    monPtr->setFileName(fPath);
    
    PythonConfigDict modelConfig;
    modelConfig.setConfigInt("nPoints", NPTS);
    modelConfig.setConfigInt("nActuators", NACTS+2);
    py::object model = ctrlPtr->execCommandWithReturn("ctrlCNN(nPoints, nActuators)", modelConfig);

    PythonConfigDict optimizerConfig;
    optimizerConfig.setConfigDouble("lr", 0.01);
    optimizerConfig.setConfigObject("params", model.attr("parameters")());
    py::object optimizer = ctrlPtr->execCommandWithReturn("torch.optim.Adam(params, lr=lr)", optimizerConfig);
        
    PythonConfigDict initConfig;
    initConfig.setConfigObject("model", model);
    initConfig.setConfigObject("optimizer", optimizer);
    ctrlPtr->setInitKwargs(initConfig);
    
    PythonConfigDict stepConfig;
    stepConfig.setConfigObject("model", model);
    stepConfig.setConfigObject("optimizer", optimizer);
    ctrlPtr->setStepKwargs(stepConfig);

    orch2.connectSystem();
    orch2.init();
    t = 0.0;

    for(size_t i = 0; i < nSteps*NUPDSTRAIN + 1; i++){
        t+=STPSZ;
        orch2.step(t, STPSZ);
    }

    PythonConfigDict saveConfig;
    saveConfig.setConfigObject("model", model);
    saveConfig.setConfigObject("optimizer", optimizer);
    saveConfig.setConfigString("fileName", "../results/models/trainedModel.pth"); //Is this relative path correct?
    ctrlPtr->execCommand("saveModel(model, optimizer, fileName)", saveConfig);
    //-------------------------------THIRD RUN: TEST MODEL
    fPath = "../results/reinforcementLearningTEST.h5";
    Orchestrator orch3;
    orch3.createModule("randomOpt", "opt", 0, [&lowerMask, &upperMask, &tsettle](IModule* mod){
        RandomOpt* optPtr = dynamic_cast<RandomOpt*>(mod);
        if(!optPtr){
            throw std::runtime_error("Problem casting from IModule class");
        }

        optPtr->setZ(zPts);
        optPtr->setZOut(zPts);
        optPtr->setMask(lowerMask, upperMask);
        optPtr->setPointRange(size_t(NACTS/2), NACTS*2);
        optPtr->setUpdatePeriod(tsettle*4);
        optPtr->setIType(interpType::CSP);
    });
    orch3.createModule("pythonWrapper", "ctrl", 1, pythonWrapperConfig3);
    orch3.createModule("constantAccelAct", "act", 2, constantAccelActConfig2);
    orch3.createModule("phillipsPhy", "phy", 3, phillipsPhyConfig2);
    orch3.createModule("simpleLogger", "mon", 4, simpleLogger2);
    orch3.configureModules();
    monPtr = dynamic_cast<SimpleLogger*>(orch3.getModule("mon"));
    ctrlPtr = dynamic_cast<PythonWrapper*>(orch3.getModule("ctrl"));

    monPtr->setFileName(fPath);
    
    PythonConfigDict loadConfig;
    loadConfig.setConfigInt("nPoints", NPTS);
    loadConfig.setConfigInt("nActuators", NACTS + 2);
    loadConfig.setConfigString("fileName", "../results/models/trainedModel.pth"); //Is this relative path correct?
    py::object loadedModel = ctrlPtr->execCommandWithReturn("loadModel(fileName, nPoints, nActuators)", loadConfig);
    model = loadedModel[py::int_(0)];
    optimizer = loadedModel[py::int_(1)];

    initConfig.setConfigObject("model", model);
    initConfig.setConfigObject("optimizer", optimizer);
    ctrlPtr->setInitKwargs(initConfig);
    
    stepConfig.setConfigObject("model", model);
    stepConfig.setConfigObject("optimizer", optimizer);
    ctrlPtr->setStepKwargs(stepConfig);

    orch3.connectSystem();
    orch3.init();
    t = 0.0;

    for(size_t i = 0; i < nSteps*NUPDSTEST; i++){
        t+=STPSZ;
        orch3.step(t, STPSZ);
    }

}
