#ifndef PHILLIPSPHY_HPP
#define PHILLIPSPHY_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace uasisi{
//It is recommended for zOut to be phi-linspaced. The first and last elements of zOut must correspond to the tips of the wing and will be ommited from computation
double trapz(const std::vector<double>& x, const std::vector<double>& y);
double trapzProduct(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);
bool checkEdges(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z, double tol);

class PhillipsPhy : public IPhysics{

    public:

    PhillipsPhy();
    ~PhillipsPhy() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& name, SpanwiseVec<double>* x) override; 
    void connectOutputScalar(const std::string& name, double* x) override;// I just added the SCA type in DataType and the virtual functions in module.hpp. I am wondering now about modules like the arbibtrary optimizer. Would it not make more sense to have l and k and stuff as scalar input signals rather than setup by the orchestrator? In this case it makes a lot of sense to use a scalar signal since this is the sort of thing that I need to be able to log and plot. Not sure about the optimizer though. For now it might be okay, perhaps a more advanced optimizer can be written later on.
    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override;
    void connectInputAirfoil(const std::string& name, const SpanwiseVec<airfoil>* x) override;
    
    void step(double t, double dt) override;
    void updateFConds(const FlightConditions& fCondsNew); //CAN change flight conditions during simulation but needs to reinit module
    void refreshZ();
    void refreshData();
    void computeRe();
    void obtainPolars(); // For now just return constant values

    const std::string& getName() const override{return this->name;}

    void setZOut(const std::vector<double>& zNew);
    void setAlpha0(const SpanwiseVec<double>& alphaNew);
    void setChord(const SpanwiseVec<double>& chordNew);
    void setIType(const interpType& tNew);
    void setFoil(const SpanwiseVec<airfoil>& foilNew);
    void setTwist(const SpanwiseVec<double>& twistNew);
    void setDefaultFoil(const std::string& nameNew);
    void setDefaultTwist(double twistNew);
    void setDefaultChord(double chordNew);
    void setTolerance(double tolNew);
    void setNPoints(size_t nNew);
    void setNSWPoints(size_t nNew);

    private:

    void translateDomain();
    void generateFij();
    void computeCoeffs();
    void computeA();
    void computeB();
    void computeLift();
    //void getLinearRegion(const std::vector<double>& x, const std::vector<double>& phi); Will implement this when I need to write an actual obtainPolars() method.
    void readZTwist();
    void readZGeo();
    void readTwist();
    void readGeo();

    std::string name = "phillipsPhy"; //I need to check every module has the same camel case format
   
    double span;
    double tolerance = 1e-10;
    std::vector<double> chord;
    std::vector<double> lSlope;
    std::string defaultFoil = "NACA2412";
    double defaultTwist = 0.0;
    double defaultChord = 1.0;
    size_t nPoints = 100;
    size_t nSWPoints;

    FlightConditions fConds;
    std::vector<double> Re;

    Eigen::MatrixXd Fij; //Im not sure if there is a way to have a non dynamic matrix. The size is only dependant on the number of points. Maybe with the constructor? Sadly, I do not think it is possible
    Eigen::MatrixXd invFij;
    Eigen::VectorXd alpha0;
    Eigen::VectorXd alpha0L;
    Eigen::VectorXd A;
    Eigen::VectorXd B;

    
    std::vector<double> zTwist;
    std::vector<double> zGeo;
    std::vector<double> phi;
    std::vector<double> phiZ;
    std::vector<double> zOut; //Must be set even if only scalar outputs are used

    std::vector<double> interpTwist;
    std::vector<airfoil> interpGeo;
    Eigen::VectorXd interpTwistEigen;

    bool fCondsIsSet = false; //The class has defaults so this check should just print a warning
    bool zOutIsSet = false;
    bool zIsSet = false; //This module will not read z after init until told to. This is for efficiency
    bool dataIsSet = false;
    bool ReIsSet = false;
    bool PhiIsSet = false;
    bool polarsAreSet = false;
    bool spanIsSet = false;
    bool FijIsSet = false;
    bool iTypeIsSet = false;
    bool alpha0IsSet = false;
    bool chordIsSet = false;
    bool foilIsSet = false;
    bool twistIsSet = false;
    bool AIsSet = false;
    bool BIsSet = false;
    bool nSWPointsIsSet = false;

    bool domainIsSet = false;
    bool isConnected = false;
    bool rTwistIsConnected = false;
    bool rGeometryIsConnected = false;
    bool rLiftIsConnected = false;
    bool rTLiftIsConnected = false;
    bool rRMomentIsConnected = false;
    bool isSet = false;

    bool airfoilProvided = false;

    DataType dType = DataType::DOB;
    interpType iType;

    const SpanwiseVec<double>* realTwist = nullptr;
    const SpanwiseVec<airfoil>* realGeometry = nullptr;

    SpanwiseVec<double>* realLift = nullptr;
    double* realTotalLift = nullptr;
    double* realRollingMoment = nullptr;

};

}

#endif
