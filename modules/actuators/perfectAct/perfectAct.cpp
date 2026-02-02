#include "perfectAct.hpp"
#include <iostream>
#include <stdexcept>

namespace uasisi {

PerfectAct::PerfectAct(){
 std::cout << "Perfect Actuator created\n";
}

void PerfectAct::init(){
    
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->dTypeIsSet || !this->iTypeIsSet){
        throw std::runtime_error("Error, Perfect Actuator Module not fully setup");
    }
    if(this->isConnected){
        if(this->rGeometryConnected){
            if(this->initialStateIsSet){
                if(this->dType == DataType::DOB && this->z.size() == this->targetGeometryDOB->size()){
                    if(this->addThickness){
                        if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
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
                } else if(this->dType == DataType::VEC || this->z.size() == this->targetGeometryVEC->size()){
                    if(this->addThickness){
                        if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
                            throw std::runtime_error("Module not configured for thickness");
                        }
                        std::vector<double> zThick;
                        std::vector<std::vector<double>> dataThick;

                        double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                        double zTemp = this->z[0];

                        zThick.reserve(2*this->z.size());
                        dataThick.reserve(2*this->z.size());
                        zThick.push_back(this->z[0]);
                        dataThick.push_back(this->initialStateVEC[0]);
                        zThick.push_back(zThick[0] + this->thickness);
                        dataThick.push_back(this->initialStateVEC[0]);

                        for(size_t i = 1; i < (z.size() - 1); i++){
                            zTemp += dz;
                            zThick.push_back(zTemp - this->thickness/2);
                            dataThick.push_back(this->initialStateVEC[i]); //Check size fit on setter method
                            zThick.push_back(zTemp + this->thickness/2);
                            dataThick.push_back(this->initialStateVEC[i]);
                        }

                        zThick.push_back(z[this->z.size()-1] - this->thickness);
                        zThick.push_back(z[this->z.size()-1]);
                        dataThick.push_back(this->initialStateVEC[this->z.size() - 1]);
                        dataThick.push_back(this->initialStateVEC[this->z.size() - 1]);

                        this->realGeometryVEC->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType)));
                    } else {
                        this->realGeometryVEC->set(zOut, uasisi::interpolate(this->z, this->initialStateVEC, this->zOut, getInterpType(this->iType)));
                    }
                } else if (this->dType == DataType::AIR || this->z.size() == this->targetGeometryAIR->size()){
                    if(this->addThickness){
                        if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
                            throw std::runtime_error("Module not configured for thickness");
                        }
                        std::vector<double> zThick;
                        std::vector<airfoil> dataThick;

                        double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                        double zTemp = this->z[0];

                        zThick.reserve(2*this->z.size());
                        dataThick.reserve(2*this->z.size());
                        zThick.push_back(this->z[0]);
                        dataThick.push_back(this->initialStateAIR[0]);
                        zThick.push_back(zThick[0] + this->thickness);
                        dataThick.push_back(this->initialStateAIR[0]);

                        for(size_t i = 1; i < (z.size() - 1); i++){
                            zTemp += dz;
                            zThick.push_back(zTemp - this->thickness/2);
                            dataThick.push_back(this->initialStateAIR[i]); //Check size fit on setter method
                            zThick.push_back(zTemp + this->thickness/2);
                            dataThick.push_back(this->initialStateAIR[i]);
                        }

                        zThick.push_back(z[this->z.size()-1] - this->thickness);
                        zThick.push_back(z[this->z.size()-1]);
                        dataThick.push_back(this->initialStateAIR[this->z.size() - 1]);
                        dataThick.push_back(this->initialStateAIR[this->z.size() - 1]);

                        this->realGeometryAIR->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType)));
                    } else {
                        this->realGeometryAIR->set(zOut, uasisi::interpolate(this->z, this->initialStateAIR, this->zOut, getInterpType(this->iType)));
                    }
                } else throw std::runtime_error("Invalid type or initial state size mismatch");
            } else { //Kind of undefined behaviour for the first step. The orchestrator should not step modules that depend on this one until this one has been stepped
                throw std::runtime_error("At the moment, this module requires an initial state to be set"); //Maybe later add a more complex solution
            }
        }
    } else {
        throw std::runtime_error("Module unconnected");
    }

    this->isSet = true;

}

std::vector<SignalInfo> PerfectAct::declareSignals(){
    if(!this->dTypeIsSet){
        throw std::runtime_error("Data type is not set");
    }
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(2);
    reqSignals.push_back(SignalInfo("targetGeometry", this->dType, SignalType::IN));
    reqSignals.push_back(SignalInfo("realGeometry", this->dType, SignalType::OUT));

    return reqSignals;
}

void PerfectAct::validateConnections(){
    
    if(!this->dTypeIsSet){
        throw std::runtime_error("Validation failed. Make sure dType is set");
    }
    if(!((dType == DataType::DOB && this->targetGeometryDOB && this->realGeometryDOB) || (dType == DataType::VEC && this->targetGeometryVEC && this->realGeometryVEC) || (dType == DataType::AIR && this->targetGeometryAIR && this->realGeometryAIR))){
        std::cout << "WARNING: some signals might be unconnected\n";
        if(this->targetGeometryDOB || this->targetGeometryVEC || this->targetGeometryAIR){ //Only input connected
            if(this->inputs.size() != 1 || this->inputs[0].name() != "targetGeometry"){ 
                throw std::runtime_error("Validation failed");
            }
        } else if (this->realGeometryDOB || this->realGeometryVEC || this->realGeometryAIR){ // Only output connected // the validation logic is getting more complex with each module. Its still ok but maybe it can be further normalised. I need to think about this.
            if(this->outputs.size() != 1 || this->outputs[0].name() != "realGeometry"){ 
                throw std::runtime_error("Validation failed");
            }
        }
    } else {
        if(this->outputs.size() != 1 || this->inputs.size() != 1 || this->outputs[0].name() != "realGeometry" || this->inputs[0].name() != "targetGeometry"){ 
            throw std::runtime_error("Validation failed");
        }
    }
    if(dType == DataType::DOB){
        if(targetGeometryDOB) this->tGeometryConnected = true;
        if(realGeometryDOB){
            if(!this->zOutIsSet){
                throw std::runtime_error("zOut not set"); 
            }
            this->rGeometryConnected = true;
        }
    } else if(dType == DataType::VEC){
        if(targetGeometryVEC) this->tGeometryConnected = true;
        if(realGeometryVEC){
            if(!this->zOutIsSet){
                throw std::runtime_error("zOut not set"); 
            }
            this->rGeometryConnected = true;
        }
    } else if(dType == DataType::AIR){
        if(targetGeometryDOB){
            if(!this->zOutIsSet){
                throw std::runtime_error("zOut not set"); 
            }
            this->tGeometryConnected = true;
        }
        if(realGeometryAIR){
            if(!this->zOutIsSet){
                throw std::runtime_error("zOut not set"); 
            }
            this->rGeometryConnected = true;
        }
    } else {
        throw std::runtime_error("Invalid data type");
    }
    this->isConnected = true;

}

void PerfectAct::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){

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

void PerfectAct::connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x){

    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realGeometry"){
        this->realGeometryVEC = x;
        this->outputs.push_back(SignalInfo(name, DataType::VEC, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectAct::connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x){
    
    if(!this->dTypeIsSet || this->dType != DataType::AIR){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "realGeometry"){
        this->realGeometryAIR = x;
        this->outputs.push_back(SignalInfo(name, DataType::AIR, SignalType::OUT));
        std::cout << "Connected output: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectAct::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){ //Does the const really prevent the module from writing to the SWVec? not sure

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

void PerfectAct::connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x){

    if(!this->dTypeIsSet || this->dType != DataType::VEC){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryVEC = x;
        this->inputs.push_back(SignalInfo(name, DataType::VEC, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectAct::connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x){
    
    if(!this->dTypeIsSet || this->dType != DataType::AIR){
        throw std::runtime_error("Unset or wrong data type");
    }
    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(name == "targetGeometry"){
        this->targetGeometryAIR = x;
        this->inputs.push_back(SignalInfo(name, DataType::AIR, SignalType::IN));
        std::cout << "Connected input: " << name << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + name);
    }

}

void PerfectAct::step(double t, double dt){
    if(this->tGeometryConnected && this->rGeometryConnected){
        if(this->dType == DataType::DOB){
            if(this->addThickness){
                if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
                    throw std::runtime_error("Module not configured for thickness");
                }
                std::vector<double> zThick;
                std::vector<double> dataThick;

                this->z = this->targetGeometryDOB->coords(); //I know copying is very slow but I just cba right now
                this->initialStateDOB = this->targetGeometryDOB->SWData(); //I need to remember to add an operator or method to the SWData class to return an indexed coord instead of an indexed data element. Seems obvious that I missed it and it will be needed to avoid lazy copying like this.

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

                this->realGeometryDOB->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType))); //I have been using the interpolate function a lot instead of the built in method. I need to sit down to think about the most efficient way to write these modules, some best practices. All of these are pretty janky
            } else {
                this->realGeometryDOB->set(zOut, uasisi::interpolate(this->z, this->initialStateDOB, this->zOut, getInterpType(this->iType))); //Is set the best option here? this is not the init function. Maybe the values should be changed individually. Im pretty sure that is not a good way to approach it but still worth thinking about.
            }
        } else if(this->dType == DataType::VEC){
            if(this->addThickness){
                if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
                    throw std::runtime_error("Module not configured for thickness");
                }
                std::vector<double> zThick;
                std::vector<std::vector<double>> dataThick;
                
                this->z = this->targetGeometryVEC->coords(); 
                this->initialStateVEC = this->targetGeometryVEC->SWData();

                double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                double zTemp = this->z[0];

                zThick.reserve(2*this->z.size());
                dataThick.reserve(2*this->z.size());
                zThick.push_back(this->z[0]);
                dataThick.push_back(this->initialStateVEC[0]);
                zThick.push_back(zThick[0] + this->thickness);
                dataThick.push_back(this->initialStateVEC[0]);

                for(size_t i = 1; i < (z.size() - 1); i++){
                    zTemp += dz;
                    zThick.push_back(zTemp - this->thickness/2);
                    dataThick.push_back(this->initialStateVEC[i]); //Check size fit on setter method
                    zThick.push_back(zTemp + this->thickness/2);
                    dataThick.push_back(this->initialStateVEC[i]);
                }

                zThick.push_back(z[this->z.size()-1] - this->thickness);
                zThick.push_back(z[this->z.size()-1]);
                dataThick.push_back(this->initialStateVEC[this->z.size() - 1]);
                dataThick.push_back(this->initialStateVEC[this->z.size() - 1]);

                this->realGeometryVEC->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType)));
            } else {
                this->realGeometryVEC->set(zOut, uasisi::interpolate(this->z, this->initialStateVEC, this->zOut, getInterpType(this->iType))); //
            }
        } else if(this->dType == DataType::AIR){
            if(this->addThickness){
                if(!this->thicknessIsSet || !(this->thickness > 0) || this->z.size() < 3){
                    throw std::runtime_error("Module not configured for thickness");
                }
                std::vector<double> zThick;
                std::vector<airfoil> dataThick;
                
                this->z = this->targetGeometryAIR->coords(); 
                this->initialStateAIR = this->targetGeometryAIR->SWData();

                double dz = (this->z[this->z.size()-1] - this->z[0]) / (this->z.size() - 1);
                double zTemp = this->z[0];

                zThick.reserve(2*this->z.size());
                dataThick.reserve(2*this->z.size());
                zThick.push_back(this->z[0]);
                dataThick.push_back(this->initialStateAIR[0]);
                zThick.push_back(zThick[0] + this->thickness);
                dataThick.push_back(this->initialStateAIR[0]);

                for(size_t i = 1; i < (z.size() - 1); i++){
                    zTemp += dz;
                    zThick.push_back(zTemp - this->thickness/2);
                    dataThick.push_back(this->initialStateAIR[i]); //Check size fit on setter method
                    zThick.push_back(zTemp + this->thickness/2);
                    dataThick.push_back(this->initialStateAIR[i]);
                }

                zThick.push_back(z[this->z.size()-1] - this->thickness);
                zThick.push_back(z[this->z.size()-1]);
                dataThick.push_back(this->initialStateAIR[this->z.size() - 1]);
                dataThick.push_back(this->initialStateAIR[this->z.size() - 1]);

                this->realGeometryAIR->set(zOut, uasisi::interpolate(zThick, dataThick, this->zOut, getInterpType(this->iType)));
            } else {
                this->realGeometryAIR->set(zOut, uasisi::interpolate(this->z, this->initialStateAIR, this->zOut, getInterpType(this->iType)));
            }
        } else {
            throw std::runtime_error("Invalid data type");
        }
    }

}

void PerfectAct::setZ(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->z = zNew;
    this->zIsSet = true;
}

void PerfectAct::setZOut(const std::vector<double>& zNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->zOut = zNew;
    this->zOutIsSet = true;
}

void PerfectAct::setThickness(const double& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->thickness = tNew;
    this->thicknessIsSet = true;
}

void PerfectAct::setInitialStateDOB(const std::vector<double>& x0New){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::DOB || this->initialStateIsSet){
        throw std::runtime_error("Data type mismatch or module already has an initial state");
    }
    this->initialStateDOB = x0New;
    this->initialStateIsSet = true;
}

void PerfectAct::setInitialStateVEC(const std::vector<std::vector<double>>& x0New){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::VEC || this->initialStateIsSet){
        throw std::runtime_error("Data type mismatch or module already has an initial state");
    }
    this->initialStateVEC = x0New;
    this->initialStateIsSet = true;
}

void PerfectAct::setInitialStateAIR(const std::vector<airfoil>& x0New){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(!this->dTypeIsSet || this->dType != DataType::AIR || this->initialStateIsSet){
        throw std::runtime_error("Data type mismatch or module already has an initial state");
    }
    this->initialStateAIR = x0New;
    this->initialStateIsSet = true;
}

void PerfectAct::setDType(const DataType& tNew){ //My vision for this simulator is that on the main source file, the configuration will stay somewhat constant and there will be helper setup functions for each modules which call all the appropriate setters.
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(this->dTypeIsSet){
        throw std::runtime_error("Data type already set");
    }
    this->dType = tNew;
    this->dTypeIsSet = true;
}

void PerfectAct::setIType(const interpType& tNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->iType = tNew;
    this->iTypeIsSet = true;
}

void PerfectAct::setAddThickness(bool cond){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->addThickness = cond;
}

}
namespace{
static uasisi::ModuleRegistration<uasisi::PerfectAct> registrationPerfectAct("perfectAct");
}
