#include "pythonWrapper.hpp"
#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/utils/pybindhelper.hpp"
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/stl.h"
#include <ios>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>

using namespace py::literals;

namespace uasisi{

PythonWrapper::PythonWrapper(){
    std::cout << "Python Wrapper created\n";
}

void PythonWrapper::init(){
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!this->scriptIsLoaded){
        throw std::runtime_error("Script not loaded");
    }
    if(!this->isConnected){
        throw std::runtime_error("Module not connected");
    }

    if(this->isEmpty){
        std::cout << "WARNING: PythonWrapper with no signals. Nothing will be executed\n";
        return;
    }

    this->updateInputDict();
    this->outputDict = py::dict(); //Fresh dictionary
    this->outputDict = this->initFunc(this->inputDict, **this->initKwargs);
    this->readOutputDict();

    this->isSet = true;    
}

std::vector<SignalInfo> PythonWrapper::declareSignals(){
    if(!this->scriptIsLoaded){
        throw std::runtime_error("Script is not loaded");
    }
    if(!this->declareSignalsFunc){
        throw std::runtime_error("No matching function defined");
    }

    py::object signals = this->declareSignalsFunc(**this->declareSignalsKwargs);
    py::list signalList = signals.cast<py::list>();
    std::vector<SignalInfo> requestedSignals;
    if(signalList.empty()){
        std::cout << "WARNING: No signals requested\n";
        return requestedSignals; //return empty vector
    }

    py::dict signalDict;
    std::string name;
    std::string dataTypeString;
    std::string signalTypeString;
    DataType dataType;
    SignalType signalType;

    for(auto item : signalList){
        signalDict = item.cast<py::dict>();
        if(!signalDict.contains("name") || !signalDict.contains("dType") || !signalDict.contains("sType")){
            throw std::runtime_error("Invalid dict structure");
        }
        name = signalDict["name"].cast<std::string>();
        dataTypeString = signalDict["dType"].cast<std::string>();
        signalTypeString = signalDict["sType"].cast<std::string>();


        if(dataTypeString == "DOB"){
            dataType = DataType::DOB;
        } else if (dataTypeString == "VEC") {
            dataType = DataType::VEC;
        } else if (dataTypeString == "AIR") {
            dataType = DataType::AIR;
        } else if (dataTypeString == "SCA") {
            dataType = DataType::SCA;
        } else {
            throw std::runtime_error("Unrecognized data type: " + dataTypeString);
        }
        
        if(signalTypeString == "IN"){
            signalType = SignalType::IN;
        } else if (signalTypeString == "OUT") {
            signalType = SignalType::OUT;
        } else if (signalTypeString == "BUF") {
            signalType = SignalType::BUF;
        } else {
            throw std::runtime_error("Unrecognized signal type: " + signalTypeString);
        }
       
        requestedSignals.push_back(SignalInfo(name, dataType, signalType));
    }
    return requestedSignals;
}

void PythonWrapper::validateConnections(){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or set");
    }
    if(!this->dataDirInDOB.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirInDOB){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->inputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::DOB && signalInfo.signalType() == SignalType::IN){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirInVEC.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirInVEC){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->inputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::VEC && signalInfo.signalType() == SignalType::IN){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirInAIR.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirInAIR){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->inputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::AIR && signalInfo.signalType() == SignalType::IN){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirInSCA.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirInSCA){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->inputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::SCA && signalInfo.signalType() == SignalType::IN){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirOutDOB.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirOutDOB){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->outputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::DOB && signalInfo.signalType() == SignalType::OUT){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirOutVEC.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirOutVEC){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->outputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::VEC && signalInfo.signalType() == SignalType::OUT){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }

    }
    if(!this->dataDirOutAIR.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirOutAIR){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->outputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::AIR && signalInfo.signalType() == SignalType::OUT){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }
    if(!this->dataDirOutSCA.empty()){
        this->isEmpty = false;
        bool signalExists;
        for(const auto& [signalName, _] : this->dataDirOutSCA){
            signalExists = false;
            for(const SignalInfo& signalInfo : this->outputs){
                if(signalInfo.name() == signalName && signalInfo.dataType() == DataType::SCA && signalInfo.signalType() == SignalType::OUT){
                    signalExists = true;
                }
            }
            if(!signalExists){
                throw std::runtime_error("Hanging signal: " + signalName);
            }
        }
    }

    this->isConnected = true;
}

void PythonWrapper::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::DOB;});
    if(cond || this->dataDirInDOB.find(name) != this->dataDirInDOB.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirInDOB[name] = x;
    this->inputs.push_back(SignalInfo(name, DataType::DOB, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::VEC;});
    if(cond || this->dataDirInVEC.find(name) != this->dataDirInVEC.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirInVEC[name] = x;
    this->inputs.push_back(SignalInfo(name, DataType::VEC, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::AIR;});
    if(cond || this->dataDirInAIR.find(name) != this->dataDirInAIR.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirInAIR[name] = x;
    this->inputs.push_back(SignalInfo(name, DataType::AIR, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectInputScalar(const std::string& name, const double* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::SCA;});
    if(cond || this->dataDirInSCA.find(name) != this->dataDirInSCA.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirInSCA[name] = x;
    this->inputs.push_back(SignalInfo(name, DataType::SCA, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectOutputDouble(const std::string& name, SpanwiseVec<double>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::DOB;});
    if(cond || this->dataDirOutDOB.find(name) != this->dataDirOutDOB.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirOutDOB[name] = x;
    this->outputs.push_back(SignalInfo(name, DataType::DOB, SignalType::OUT));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::VEC;});
    if(cond || this->dataDirOutVEC.find(name) != this->dataDirOutVEC.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirOutVEC[name] = x;
    this->outputs.push_back(SignalInfo(name, DataType::VEC, SignalType::OUT));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::AIR;});
    if(cond || this->dataDirOutAIR.find(name) != this->dataDirOutAIR.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirOutAIR[name] = x;
    this->outputs.push_back(SignalInfo(name, DataType::AIR, SignalType::OUT));
    std::cout << "Connected input: " << name << std::endl;
}
void PythonWrapper::connectOutputScalar(const std::string& name, double* x){
    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::SCA;});
    if(cond || this->dataDirOutSCA.find(name) != this->dataDirOutSCA.end()){
        throw std::runtime_error("Duplicate signal");
    }
    this->dataDirOutSCA[name] = x;
    this->outputs.push_back(SignalInfo(name, DataType::SCA, SignalType::OUT));
    std::cout << "Connected input: " << name << std::endl;
}

void PythonWrapper::step(double t, double dt){
    if(!this->isSet || !this->isConnected){
        throw std::runtime_error("Module not initialized/connected");
    }
    if(this->isEmpty){
        return;
    }
    this->updateInputDict();
    this->outputDict = py::dict(); //Fresh dictionary
    this->outputDict = this->stepFunc("t"_a=t, "dt"_a=dt, "inputs"_a=this->inputDict, **this->stepKwargs);
    this->readOutputDict();
}

void PythonWrapper::updateInputDict(){
    if(this->isEmpty){
        return;
    }
    this->inputDict = py::dict(); //Fresh dictionary
    if(!this->dataDirInDOB.empty()){
        py::dict signalsDOB;
        for(const auto& [name, ptr] : this->dataDirInDOB){
            signalsDOB[name.c_str()] = dobToDic(*ptr);
        }
        this->inputDict["DOB"] = signalsDOB;
    }
    if(!this->dataDirInVEC.empty()){
        py::dict signalsVEC;
        for(const auto& [name, ptr] : this->dataDirInVEC){
            signalsVEC[name.c_str()] = vecToDic(*ptr);
        }
        this->inputDict["VEC"] = signalsVEC;
    }
    if(!this->dataDirInAIR.empty()){
        py::dict signalsAIR;
        for(const auto& [name, ptr] : this->dataDirInAIR){
            signalsAIR[name.c_str()] = airToDic(*ptr);
        }
        this->inputDict["AIR"] = signalsAIR;
    }
    if(!this->dataDirInSCA.empty()){
        py::dict signalsSCA;
        for(const auto& [name, ptr] : this->dataDirInSCA){
            signalsSCA[name.c_str()] = scaToDic(*ptr);
        }
        this->inputDict["SCA"] = signalsSCA;
    }
}

void PythonWrapper::readOutputDict(){
    if(this->isEmpty){
        return;
    }
    py::dict groupDict;
    py::dict signalDict;
    std::string signalName;
    std::string groupName;
    for(const auto& signalGroups : this->outputDict){
        groupName = py::str(signalGroups.first);
        groupDict = signalGroups.second.cast<py::dict>();
        if(groupName == "DOB"){
            for(const auto& signal : groupDict){
                signalName = py::str(signal.first);
                if(this->dataDirOutDOB.find(signalName) == this->dataDirOutDOB.end()){
                    continue;
                }
                signalDict = signal.second.cast<py::dict>();
                updateDOBWithDic(signalDict, this->dataDirOutDOB[signalName]);
            }
        }
        if(groupName == "VEC"){
            for(const auto& signal : groupDict){
                signalName = py::str(signal.first);
                if(this->dataDirOutVEC.find(signalName) == this->dataDirOutVEC.end()){
                    continue;
                }
                signalDict = signal.second.cast<py::dict>();
                updateVECWithDic(signalDict, this->dataDirOutVEC[signalName]);
            }
        }
        if(groupName == "AIR"){
            for(const auto& signal : groupDict){
                signalName = py::str(signal.first);
                if(this->dataDirOutAIR.find(signalName) == this->dataDirOutAIR.end()){
                    continue;
                }
                signalDict = signal.second.cast<py::dict>();
                updateAIRWithDic(signalDict, this->dataDirOutAIR[signalName]);
            }
        }
        if(groupName == "SCA"){
            for(const auto& signal : groupDict){
                signalName = py::str(signal.first);
                if(this->dataDirOutSCA.find(signalName) == this->dataDirOutSCA.end()){
                    continue;
                }
                signalDict = signal.second.cast<py::dict>();
                updateSCAWithDic(signalDict, this->dataDirOutSCA[signalName]);
            }
        }
    }
}

void PythonWrapper::setScriptPath(const std::string& pathNew){
    if(this->isSet || this->isConnected || this->scriptIsLoaded){
        throw std::runtime_error("Module already connected or running");
    }
    if(pathNew.empty()){
        throw std::runtime_error("Empty path received");
    }
    std::filesystem::path scriptPath(pathNew);
    if(!std::filesystem::exists(scriptPath)){
        throw std::runtime_error(pathNew + " Not found");
    }
    this->filePath = scriptPath;
    this->scriptPathIsSet = true;
    
}

void PythonWrapper::loadScript(){
    if(this->isSet || this->isConnected || this->scriptIsLoaded){
        throw std::runtime_error("Module already connected or running");
    }
    if(!this->scriptPathIsSet){
        throw std::runtime_error("Script path not set yet");
    }
    if(!std::filesystem::exists(this->filePath)){ //Checking again
        throw std::runtime_error("Script not found");
    }
    std::string scriptDir = this->filePath.parent_path().string();
    std::string scriptName = this->filePath.stem().string();

    py::module_ sys = py::module_::import("sys");
    py::list path = sys.attr("path");

    bool dirInPath = false;
    for(auto item : path){
        if(item.cast<std::string>() == scriptDir){
            dirInPath = true;
            break;
        }
    }
    if(!dirInPath){
        path.insert(0, scriptDir);
    }
    this->pythonModule = py::module_::import(scriptName.c_str());
    if(!py::hasattr(this->pythonModule, "declareSignals")){
        throw std::runtime_error("No declareSignals function defined in script");
    }
    if(!py::hasattr(this->pythonModule, "init")){
        throw std::runtime_error("No init function defined in script");
    }
    if(!py::hasattr(this->pythonModule, "step")){
        throw std::runtime_error("No step function defined in script");
    }
    this->declareSignalsFunc = pythonModule.attr("declareSignals");
    this->initFunc = pythonModule.attr("init");
    this->stepFunc = pythonModule.attr("step");
    this->scriptIsLoaded = true;
}

void PythonWrapper::setDeclareSignalsKwargs(const PythonConfigDict& kwargsNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already running/connected");
    }
    this->declareSignalsKwargs = kwargsNew.getDict();
}

void PythonWrapper::setInitKwargs(const PythonConfigDict& kwargsNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already running/connected");
    }
    this->initKwargs = kwargsNew.getDict();
}

void PythonWrapper::setStepKwargs(const PythonConfigDict& kwargsNew){
    this->stepKwargs = kwargsNew.getDict();
}


void PythonWrapper::execCommand(const std::string& command, const PythonConfigDict& kwargs){
    py::object scope;
    if(this->scriptIsLoaded){
        scope = this->pythonModule.attr("__dict__");
    } else {
        scope = py::module_::import("__main__").attr("__dict__");
    }
    py::exec(command, scope, kwargs.getDict());
}

void PythonWrapper::execSnippet(const std::string& snippet){
    py::object scope;
    if(this->scriptIsLoaded){
        scope = this->pythonModule.attr("__dict__");
    } else {
        scope = py::module_::import("__main__").attr("__dict__");
    }
    py::exec(snippet, scope);
}

void PythonWrapper::execScript(const std::string& scriptPath){
    py::object scope;
    if(this->scriptIsLoaded){
        scope = this->pythonModule.attr("__dict__");
    } else {
        scope = py::module_::import("__main__").attr("__dict__");
    }
    py::eval_file(scriptPath, scope);
}

py::object PythonWrapper::execCommandWithReturn(const std::string& command, const PythonConfigDict& kwargs){
    py::object scope;
    if(this->scriptIsLoaded){
        scope = this->pythonModule.attr("__dict__");
    } else {
        scope = py::module_::import("__main__").attr("__dict__");
    }
    return py::eval(command, scope, kwargs.getDict());
}

py::dict dobToDic(const SpanwiseVec<double>& data){
    py::dict dic;
    if(data.size() == 0){
       return dic; //return empty dictionary
    }
    dic["coords"] = py::cast(data.coords());
    dic["data"] = py::cast(data.SWData());
    dic["type"] = "DOB";
    return dic;
}

py::dict vecToDic(const SpanwiseVec<std::vector<double>>& data){
    py::dict dic;
    if(data.size() == 0){
       return dic;
    }
    dic["coords"] = py::cast(data.coords());
    dic["data"] = py::cast(data.SWData());
    dic["type"] = "VEC";
    return dic;
}

py::dict airToDic(const SpanwiseVec<airfoil>& data){
    py::dict dic;
    if(data.size() == 0){
       return dic; 
    }
    dic["coords"] = py::cast(data.coords());

    py::list foils;
    for(const airfoil& foil : data.SWData()){
        foils.append(airfoilToDic(foil));
    }

    dic["data"] = foils;
    dic["type"] = "AIR";
    return dic;
}

py::dict scaToDic(double data){ //This function is very redundant but I will add it for completeness
    py::dict dic;
    dic["data"] = data;
    dic["type"] = "SCA";
    return dic;

}

py::dict airfoilToDic(const airfoil& data){
    py::dict dic;
    if(data.size() == 0){
       return dic; 
    }
    dic["name"] = data.name();
    dic["chord"] = data.getChord();
    dic["attachmentPoint"] = pointToDic(data.getAttatchmentPoint());
    
    py::list points;
    for(const point& pt : data.coords()){
        points.append(pointToDic(pt));
    }

    dic["points"] = points;
    dic["type"] = "airfoil";
    return dic;
}

py::dict pointToDic(const point& data){
    py::dict dic;
    dic["x"] = data.x;
    dic["y"] = data.y;
    dic["z"] = data.z;
    dic["idx"] = data.idx;
    dic["type"] = "point";
    return dic;
}

SpanwiseVec<double> dicToDOB(const py::dict& data){
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "DOB"){
        throw std::runtime_error("Invalid data type for DOB converter");
    }
    return SpanwiseVec<double>(data["coords"].cast<std::vector<double>>(), data["data"].cast<std::vector<double>>());
}

SpanwiseVec<std::vector<double>> dicToVEC(const py::dict& data){
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "VEC"){
        throw std::runtime_error("Invalid data type for VEC converter");
    }
    return SpanwiseVec<std::vector<double>>(data["coords"].cast<std::vector<double>>(), data["data"].cast<std::vector<std::vector<double>>>());
}

SpanwiseVec<airfoil> dicToAIR(const py::dict& data){
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "AIR"){
        throw std::runtime_error("Invalid data type for AIR converter");
    }
    
    std::vector<airfoil> foilVec;
    py::list foils = data["data"];
    if(foils.empty()){ // I want to check if the data and coords dicts match in size but that will be handled at instantiating the return object.
        return SpanwiseVec<airfoil>(data["coords"].cast<std::vector<double>>(), foilVec);
    }
    py::dict foilDict;
    for(const auto& foil : foils){
        foilDict = foil.cast<py::dict>();
        foilVec.push_back(dicToAirfoil(foilDict));
    }
    return SpanwiseVec<airfoil>(data["coords"].cast<std::vector<double>>(), foilVec);
}

double dicToSCA(const py::dict& data){
    if(!data.contains("data") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "SCA"){
        throw std::runtime_error("Invalid data type for SCA converter");
    }
    return data["data"].cast<double>(); //Do I need to cast to double here?
}

airfoil dicToAirfoil(const py::dict& data){
    if(!data.contains("name") || !data.contains("chord") || !data.contains("attachmentPoint") || !data.contains("points") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "airfoil"){
        throw std::runtime_error("Invalid data type for airfoil converter");
    }
    airfoil foil;
    foil.setName(data["name"].cast<std::string>());
    foil.setChord(data["chord"].cast<double>());
    foil.setAttatchmentPoint(dicToPoint(data["attachmentPoint"]));

    py::list points = data["points"];
    std::vector<point> pointVec;
    if(points.empty()){
        return foil;
    }
    py::dict pointDic;
    for(const auto& point : points){
        pointDic = point.cast<py::dict>();
        pointVec.push_back(dicToPoint(pointDic));
    }
    foil.setPoints(pointVec);
    return foil;
}

point dicToPoint(const py::dict& data){
    if(!data.contains("x") || !data.contains("y") || !data.contains("z") || !data.contains("idx") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "point"){
        throw std::runtime_error("Invalid data type for point converter");
    }
    point p;
    p.x = data["x"].cast<double>();
    p.y = data["y"].cast<double>();
    p.z = data["z"].cast<double>();
    p.idx = data["idx"].cast<int>();
    return p;
}

void updateDOBWithDic(const py::dict& data, SpanwiseVec<double>* ptr){ // Would it be faster to access individual vector elements instead of using set()?? in any case, I would have to rework the SW class for that.
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "DOB"){
        throw std::runtime_error("Invalid data type for DOB converter");
    }
    ptr->set(data["coords"].cast<std::vector<double>>(), data["data"].cast<std::vector<double>>());

}

void updateVECWithDic(const py::dict& data, SpanwiseVec<std::vector<double>>* ptr){
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "VEC"){
        throw std::runtime_error("Invalid data type for VEC converter");
    }
    ptr->set(data["coords"].cast<std::vector<double>>(), data["data"].cast<std::vector<std::vector<double>>>());

}

void updateAIRWithDic(const py::dict& data, SpanwiseVec<airfoil>* ptr){
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("data") || !data.contains("coords") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "AIR"){
        throw std::runtime_error("Invalid data type for AIR converter");
    }
    *ptr = dicToAIR(data); //I am so lazy to add a proper implementation now.

}

void updateSCAWithDic(const py::dict& data, double* ptr){
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("data") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "SCA"){
        throw std::runtime_error("Invalid data type for SCA converter");
    }
    *ptr = data["data"].cast<double>();
}

void updateAirfoilWithDic(const py::dict& data, airfoil* ptr){
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("name") || !data.contains("chord") || !data.contains("attachmentPoint") || !data.contains("points") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "airfoil"){
        throw std::runtime_error("Invalid data type for airfoil converter");
    }
    *ptr = dicToAirfoil(data); //Lol

}

void updatePointWithDic(const py::dict& data, point* ptr){
    if(!ptr){
        throw std::runtime_error("Null pointer");
    }
    if(!data.contains("x") || !data.contains("y") || !data.contains("z") || !data.contains("idx") || !data.contains("type")){
        throw std::runtime_error("Invalid dictionary structure");
    }
    if(data["type"].cast<std::string>() != "point"){
        throw std::runtime_error("Invalid data type for point converter");
    }
    ptr->x = data["x"].cast<double>();
    ptr->y = data["y"].cast<double>();
    ptr->z = data["z"].cast<double>();
    ptr->idx = data["idx"].cast<int>();

}

}

namespace{
static uasisi::ModuleRegistration<uasisi::PythonWrapper> registrationPythonWrapper("pythonWrapper");
}
