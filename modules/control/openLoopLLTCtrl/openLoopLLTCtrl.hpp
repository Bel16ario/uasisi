#ifndef OPENLOOPLLTCTRL_HPP
#define OPENLOOPLLTCTRL_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>

namespace uasisi{

class OpenLoopLLTCtrl : public IControl{

    public:

    OpenLoopLLTCtrl();
    ~OpenLoopLLTCtrl() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override; 
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override; 
    
    void step(double t, double dt) override;

    const std::string& getName() const override{return this->name;}

    void setZ(const std::vector<double>& zNew);
    void setActCoords(const std::vector<double>& zNew);
    void setNSWPoints(size_t nNew);

    void setVInf(double vInfNew);
    void setRho(double rhoNew);
    void setSpan(double spanNew);
    
    void setChords(const std::vector<double>& chordsNew);
    void setZeroLiftAngle(const std::vector<double>& zlAngleNew);
    void setClSlope(const std::vector<double>& clSlopeNew);
    void setThetaMax(const std::vector<double>& thetaMaxNew);
    void setThetaCenter(const std::vector<double>& thetaCenterNew);

    void setIType(const interpType& t);

    private:

    void translateDomain();
    void computeTheta();
    void computeK2();
    
    std::string name = "openLoopLLTCtrl";
        
    std::vector<double> z;
    std::vector<double> phiZ;
    std::vector<double> phi;
    std::vector<double> actCoords;
    size_t nSWPoints;

    double vInf;
    double rho;
    double span;
    
    std::vector<double> k1;
    double k2;
    std::vector<double> theta;

    std::vector<double> chords;
    std::vector<double> zlAngle;
    std::vector<double> clSlope;
    std::vector<double> thetaMax;
    std::vector<double> thetaCenter;
    
    std::vector<double> targetLiftPhi;
    
    DataType dType = DataType::DOB;
    interpType iType;

    bool zIsSet = false;
    bool phiIsSet = false;
    bool domainIsSet = false;
    bool actCoordsAreSet = false;
    bool nSWPointsIsSet = false;
    bool vInfIsSet = false;
    bool rhoIsSet = false;
    bool spanIsSet = false;
    bool chordsIsSet = false;
    bool zlAngleIsSet = false;
    bool clSlopeIsSet = false;
    bool thetaMaxIsSet = false;
    bool thetaCenterIsSet = false;
    bool dTypeIsSet = true;
    bool iTypeIsSet = false;
    bool isSet = false;
    bool isConnected = false;

    SpanwiseVec<double>* targetGeometryDOB = nullptr;
    const SpanwiseVec<double>* targetLift = nullptr;
};

}

#endif
