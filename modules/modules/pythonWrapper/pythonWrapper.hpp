#ifndef PYTHONWRAPPER_HPP
#define PYTHONWRAPPER_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/utils/pybindhelper.hpp"
#include <map>
#include <string>
#include <vector>
#include <filesystem>

namespace py = pybind11;

namespace uasisi{

py::dict dobToDic(const SpanwiseVec<double>& data); //Should I also do update methods going this way? is it even possible?
py::dict vecToDic(const SpanwiseVec<std::vector<double>>& data);
py::dict airToDic(const SpanwiseVec<airfoil>& data);
py::dict scaToDic(double data);
py::dict airfoilToDic(const airfoil& data);
py::dict pointToDic(const point& data);

SpanwiseVec<double> dicToDOB(const py::dict& data);
SpanwiseVec<std::vector<double>> dicToVEC(const py::dict& data);
SpanwiseVec<airfoil> dicToAIR(const py::dict& data);
double dicToSCA(const py::dict& data);
airfoil dicToAirfoil(const py::dict& data);
point dicToPoint(const py::dict& data);

void updateDOBWithDic(const py::dict& data, SpanwiseVec<double>* ptr);
void updateVECWithDic(const py::dict& data, SpanwiseVec<std::vector<double>>* ptr);
void updateAIRWithDic(const py::dict& data, SpanwiseVec<airfoil>* ptr);
void updateSCAWithDic(const py::dict& data, double* ptr);
void updateAirfoilWithDic(const py::dict& data, airfoil* ptr);
void updatePointWithDic(const py::dict& data, point* ptr);

class PythonWrapper : public IModule{

    public:

    PythonWrapper();

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;
    
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override;
    void connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x) override;
    void connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x) override;
    void connectInputScalar(const std::string& name, const double* x) override;
    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override;
    void connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x) override;
    void connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x) override;
    void connectOutputScalar(const std::string& name, double* x) override;

    void step(double t, double dt) override;
    void updateInputDict();
    void readOutputDict();

    const std::string& getName() const override { return this->name; }

    void setScriptPath(const std::string& pathNew);
    void loadScript();

    void setInitKwargs(const PythonConfigDict& kwargsNew);
    void setStepKwargs(const PythonConfigDict& kwargsNew);
    void setDeclareSignalsKwargs(const PythonConfigDict& kwargsNew);

    void execCommand(const std::string& command, const PythonConfigDict& kwargs); //Execute arbitrary command with kwargs.
    void execSnippet(const std::string& snippet); //Execute arbitrary python snippet
    void execScript(const std::string& scriptPath); //Execute arbitrary python script

    py::object execCommandWithReturn(const std::string& command, const PythonConfigDict& kwargs); //Execute arbitrary command with kwargs. Store return. I feel like no point in making it update at a pointer because the object could be anything.

    private:

    struct InterpreterGuard {
        InterpreterGuard() { PythonInterpreter::getInstance(); }
    };
    InterpreterGuard interpGuard;

    std::string name = "PythonWrapper";
    std::filesystem::path filePath;

    py::dict initKwargs;
    py::dict stepKwargs;
    py::dict declareSignalsKwargs;

    py::object pythonModule;
    py::function initFunc;
    py::function stepFunc;
    py::function declareSignalsFunc;

    std::map<std::string, const SpanwiseVec<double>*> dataDirInDOB;
    std::map<std::string, const SpanwiseVec<std::vector<double>>*> dataDirInVEC;
    std::map<std::string, const SpanwiseVec<airfoil>*> dataDirInAIR;
    std::map<std::string, const double*> dataDirInSCA;
    
    std::map<std::string, SpanwiseVec<double>*> dataDirOutDOB;
    std::map<std::string, SpanwiseVec<std::vector<double>>*> dataDirOutVEC;
    std::map<std::string, SpanwiseVec<airfoil>*> dataDirOutAIR;
    std::map<std::string, double*> dataDirOutSCA;

    py::dict inputDict;
    py::dict outputDict;

    bool scriptPathIsSet = false;
    bool scriptIsLoaded = false;
    bool isConnected = false;
    bool isSet = false;
    bool isEmpty = true;


};

}

#endif
