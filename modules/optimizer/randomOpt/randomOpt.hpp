#ifndef RANDOMOPT_HPP
#define RANDOMOPT_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include <string>
#include <vector>

namespace uasisi{

size_t randomNumberFrom(size_t min, size_t max);
double randomDoubleFrom(double min, double max);
std::vector<double> reorderFrom(const std::vector<double>& data, const std::vector<size_t>& order);
double linearInterp(double z0, double f0, double z1, double f1, double z);
double sampleFromTruncatedNormal(double mean, double stddev, double min, double max);

class RandomOpt : public IOptimizer {

    public:

    RandomOpt();
    ~RandomOpt() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override;

    void step(double t, double dt) override;
    
    const std::string& getName() const override{return this->name;}


    void setZ(const std::vector<double>& zNew);
    void setZOut(const std::vector<double>& zNew);
    void setMask(const std::vector<double>& mNewMin, const std::vector<double>& mNewMax);
    void setPointRange(size_t nNewMin, size_t nNewMax);
    void setUpdatePeriod(double tNew);
    void setRelativeTolerance(double tolNew);
    void setBaselineSigma(double sigmaNew);
    void setIType(const interpType& t);

    private:

    void generateDistribution();

    std::string name = "randomOpt";

    std::vector<double> z;
    std::vector<double> upperMask;
    std::vector<double> lowerMask;
    std::vector<double> zOut;
    size_t maxPoints;
    size_t minPoints; //Cannot be smaller than 3
    double updatePeriod;
    double timeSinceUpdate = 0.0;
    double tolerance;
    double relativeTolerance = 0.005;
    double baselineSigma = 1.0;
    interpType iType;

    double span;
    double maxZ;
    double minZ;

    bool zIsSet = false;
    bool maskIsSet = false;
    bool pointRangeIsSet = false;
    bool updatePeriodIsSet = false;
    bool zOutIsSet = false;
    bool iTypeIsSet = false;
    bool isSet = false;
    bool isConnected = false;
    bool tLiftConnected = false;

    SpanwiseVec<double>* targetLift = nullptr;

};

}

#endif
