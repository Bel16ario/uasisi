#ifndef CONSTANTCTRL_HPP
#define CONSTANTCTRL_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>

namespace uasisi{

class ConstantCtrl : public IControl{

    public:

    ConstantCtrl();
    ~ConstantCtrl() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    //I want to implement the signal manager such that if a a module provides a signal that no other module wants to receive it will not connect such signal. Each module should check for this in the validation method and ideally, skip computing of such a signal. For example, for a constant control, no feedback is necessary. The physics module could in theory not do anything. Monitors can then be used to force computation. Thus this module has no inputs.
    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override; //For omega or delta
    void connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x) override; //For omega and delta
    void connectOutputAirfoil(const std::string& name, SpanwiseVec<airfoil>* x) override; //For airfoil data
    
    void step(double t, double dt) override {}

    const std::string& getName() const override{return this->name;}

    void setZ(const std::vector<double>& zNew);
    void setActCoords(const std::vector<double>& zNew);
    
    void setSData(const std::vector<double>& sDataNew);
    void setVData(const std::vector<std::vector<double>>& vDataNew);
    void setAData(const std::vector<airfoil>& aDataNew);

    void setDType(const DataType& t);
    void setIType(const interpType& t);

    private:
    
    std::string name = "constantCtrl";
        
    std::vector<double> z;
    std::vector<double> actCoords;

    std::vector<double> sData;
    std::vector<std::vector<double>> vData;
    std::vector<airfoil> aData;
    
    DataType dType;
    interpType iType;
    //In other, more complex modules, individual connect flags might be needed for each signal
    bool zIsSet = false;
    bool actCoordsAreSet = false;
    bool dataIsSet = false;
    bool dTypeIsSet = false;
    bool iTypeIsSet = false;
    bool isSet = false;
    bool isConnected = false;

    SpanwiseVec<double>* targetGeometryDOB = nullptr;
    SpanwiseVec<std::vector<double>>* targetGeometryVEC = nullptr;
    SpanwiseVec<airfoil>* targetGeometryAIR = nullptr;
};

}

#endif
