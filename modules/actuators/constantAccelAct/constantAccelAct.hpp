#ifndef CONSTANTACCELACT_HPP
#define CONSTANTACCELACT_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>

namespace uasisi{
//THIS MODULE ONLY WORKS FOR TORSION ACTUATORS. THUS ONLY THE DOUBLE DATA TYPE (OMEGA) IS SUPPORTED
//a constant acceleration can be applied in both directions.
class ConstantAccelAct : public IActuator{

    public:

    ConstantAccelAct();
    ~ConstantAccelAct() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override; 
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override; 
    
    void step(double t, double dt) override;

    const std::string& getName() const override{return this->name;}
    
    void setZ(const std::vector<double>& zNew);
    void setZOut(const std::vector<double>& zNew);
    void setMaxPos(const std::vector<double>& thetaMax);
    void setMaxVel(const std::vector<double>& omegaMax);
    void setMaxAccel(const std::vector<double>& alphaMax);
    void setCenterPos(const std::vector<double>& thetaCenter);
    void setTheta(const std::vector<double>& thetaNew);
    void setOmega(const std::vector<double>& omegaNew);
    void setPosTol(const double& tolNew);
    void setThickness(const double& tNew);
    void setAddThickness(bool cond);
    
    void setIType(const interpType& tNew);

    private:
    
    std::string name = "constantAccelAct";

    std::vector<double> z; //ACTUATOR COORDS
    std::vector<double> zOut;
    std::vector<double> maxPos; //plus minus around center position
    std::vector<double> maxVel; //should be positive number. It is implied that the inverse velocity is also possible
    std::vector<double> maxAccel; //Same case
    std::vector<double> centerPos;
    std::vector<double> theta;
    std::vector<double> omega; //can be kind of comfusing because in the wider context of a morphic wing, omega is the torsion. In this case though, it is the angular velocity by convention. A similar issue occurs with alpha.
    double posTol = 0.001;
    double thickness;
        
    DataType dType = DataType::DOB;
    interpType iType;

    bool dTypeIsSet = true;
    bool iTypeIsSet = false;

    bool addThickness = false;

    bool isSet = false;
    bool zIsSet = false;
    bool zOutIsSet = false;
    bool maxVelIsSet = false;
    bool maxAccelIsSet = false;
    bool maxPosIsSet = false;
    bool centerPosIsSet = false;
    bool thetaIsSet = false;
    bool omegaIsSet = false;
    bool initialStateIsSet = false;
    bool thicknessIsSet = false; 
    bool tGeometryConnected = false;
    bool rGeometryConnected = false;
    bool isConnected = false;

    const SpanwiseVec<double>* targetGeometryDOB = nullptr;
    SpanwiseVec<double>* realGeometryDOB = nullptr;
};

}

#endif
