#ifndef PERFECTSEN_HPP
#define PERFECTSEN_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"

namespace uasisi{

class PerfectSen : public ISensor{

    public:

    PerfectSen();
    ~PerfectSen() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override;
    void connectOutputVector(const std::string& name, SpanwiseVec<std::vector<double>>* x) override;
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override;
    void connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x) override;
    
    void step(double t, double dt) override;

    const std::string& getName() const override{return this->name;}
    
    void setZ(const std::vector<double>& zNew);
    void setZOut(const std::vector<double>& zNew);
    
    void setDType(const DataType& t);
    void setIType(const interpType& t);

    private:

    std::vector<double> z;
    std::vector<double> zOut;
    
    std::string name = "perfectSen";
        
    DataType dType;
    interpType iType;

    bool dTypeIsSet = false;
    bool iTypeIsSet = false;

    bool zIsSet = false;
    bool zOutIsSet = false;
    bool isSet = false;
    bool rLiftConnected = false;
    bool sLiftConnected = false;
    bool isConnected = false;
    
    const SpanwiseVec<double>* realLiftDOB = nullptr;
    const SpanwiseVec<std::vector<double>>* realLiftVEC = nullptr;

    SpanwiseVec<double>* sensedLiftDOB = nullptr;
    SpanwiseVec<std::vector<double>>* sensedLiftVEC = nullptr;
};

}

#endif
