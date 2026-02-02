#ifndef PERFECTACT_HPP
#define PERFECTACT_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>

namespace uasisi{

class PerfectAct : public IActuator{

    public:

    PerfectAct();
    ~PerfectAct() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override; //Is a disconnect method needed? I have made these modules so that once you validate connections and initialize, you can no longer change things so it would make sense to be able to disconnect or change signals before validating. Not sure it would be very useful at this early point of developing but for completeness sake it might be worth it.

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override; //For omega or delta
    void connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x) override; //For omega and delta
    void connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x) override; //For airfoil data
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override; //For omega or delta
    void connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x) override; //For omega and delta
    void connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x) override; //For airfoil data
    
    void step(double t, double dt) override;

    const std::string& getName() const override{return this->name;}
    
    void setZ(const std::vector<double>& zNew);
    void setZOut(const std::vector<double>& zNew);
    void setThickness(const double& tNew);
    
    void setInitialStateDOB(const std::vector<double>& x0New);
    void setInitialStateVEC(const std::vector<std::vector<double>>& x0New);
    void setInitialStateAIR(const std::vector<airfoil>& x0New);

    void setDType(const DataType& tNew);
    void setIType(const interpType& tNew);
    void setAddThickness(bool cond);

    private:
    
    std::string name = "perfectAct";

    std::vector<double> z;
    std::vector<double> zOut;
    double thickness;
    std::vector<double> initialStateDOB;
    std::vector<std::vector<double>> initialStateVEC;
    std::vector<airfoil> initialStateAIR;
        
    DataType dType;
    interpType iType;

    bool dTypeIsSet = false;
    bool iTypeIsSet = false;

    bool addThickness = false;

    bool isSet = false;
    bool zIsSet = false;
    bool zOutIsSet = false;
    bool initialStateIsSet = false;
    bool thicknessIsSet = false; 
    bool tGeometryConnected = false;
    bool rGeometryConnected = false;
    bool isConnected = false;
    
    const SpanwiseVec<double>* targetGeometryDOB = nullptr;
    const SpanwiseVec<std::vector<double>>* targetGeometryVEC = nullptr;
    const SpanwiseVec<airfoil>* targetGeometryAIR = nullptr;

    SpanwiseVec<double>* realGeometryDOB = nullptr;
    SpanwiseVec<std::vector<double>>* realGeometryVEC = nullptr;
    SpanwiseVec<airfoil>* realGeometryAIR = nullptr;
};

}

#endif
