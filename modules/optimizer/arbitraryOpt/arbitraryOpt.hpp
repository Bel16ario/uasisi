#ifndef ARBITRARYOPT_HPP
#define ARBITRARYOPT_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>

namespace uasisi{

class ArbitraryOpt : public IOptimizer {

    public:

    ArbitraryOpt();
    ~ArbitraryOpt() override = default;

    void init(const Config& config) override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override;

    void step(double t, double dt) override {}
    
    const std::string& getName() const override{return this->name;}

    void setZ(const std::vector<double>& zNew);
    void setZOut(const std::vector<double>& zNew);
    void setL(const std::vector<double>& lNew);
    void setK(const double& kNew);
    void setIType(const interpType& t);

    private:

    std::string name = "arbitraryOpt";

    double k = 1000.0;
    std::vector<double> z;
    std::vector<double> l;
    std::vector<double> zOut;
    interpType iType;

    bool zIsSet = false;
    bool lIsSet = false;
    bool kIsSet = false;
    bool zOutIsSet = false;
    bool iTypeIsSet = false;
    bool isSet = false;
    bool isConnected = false;

    SpanwiseVec<double>* targetLift = nullptr;

};

}

#endif
