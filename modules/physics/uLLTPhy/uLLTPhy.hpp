#ifndef ULLTPHY_HPP
#define ULLTPHY_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>

using MapM = Eigen::Map<Eigen::MatrixXd>;
using MapCM = Eigen::Map<const Eigen::MatrixXd>;

namespace uasisi{
double trapz(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
double trapzProduct(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z);

enum class IntegratorScheme{Euler, ExpEuler, ZOH, Heun, RK4, RFK45};

struct ULLTConfig{
	size_t nPts = 101;
	double span = 10.0;
	double rho = 1.225;

	Eigen::MatrixX3d psiInit;
	Eigen::MatrixXd aWag;
	Eigen::MatrixXd lamWag;	
	Eigen::MatrixXd chiInit;
	Eigen::MatrixXd aGen;
	Eigen::MatrixXd lamGen;	
	Eigen::MatrixXd xiInit;

	double du = 0;

	double cRef = 1.0;

	IntegratorScheme integrator = IntegratorScheme::ExpEuler;
};

class ULLTPhy : public IPhysics{

    public:

    ULLTPhy();
    ~ULLTPhy() override = default;

	void configure(const ULLTConfig& config);

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectOutputDouble(const std::string& nName, SpanwiseVec<double>* x) override; 
    void connectOutputScalar(const std::string& nName, double* x) override;
    void connectInputScalar(const std::string& nName, const double* x) override;
    void connectInputDouble(const std::string& nName, const SpanwiseVec<double>* x) override;
    void connectInputAirfoil(const std::string& nName, const SpanwiseVec<airfoil>* x) override;
    
    void step(double t, double dt) override;

    const std::string& getName() const override{return this->name;}

    private:

	void calcEta();
	void calcLift();
	void calcG();
	void calcDu();
	void calcDwy(const Eigen::VectorXd& nWy);

	void algebraic(const Eigen::VectorXd& x, Eigen::VectorXd& wy, Eigen::VectorXd& alphaE, Eigen::VectorXd& lc);
	void rates(const Eigen::VectorXd& x, Eigen::VectorXd& dx);
	void integrate(double ds);

	void stepEuler(double ds);
	void stepExpEuler(double ds);
	void stepZOH(double ds);
	void stepHeun(double ds);
	void stepRK4(double ds);
	void stepRFK45(double ds);

	double tolerance = 1e-10;
    std::string name = "uLLTPhy";
    size_t nPoints = 100;

	IntegratorScheme integrator = IntegratorScheme::Euler;
	size_t nWag = 0;
	size_t nGen = 0;
	Eigen::VectorXd X;

	MapM chiOf(Eigen::VectorXd& x) const { return MapM(x.data(), this->nPoints, this->nWag);}
	MapCM chiOf(const Eigen::VectorXd& x) const { return MapCM(x.data(), this->nPoints, this->nWag);}
	MapM xiOf(Eigen::VectorXd& x) const { return MapM(x.data() + this->nPoints*this->nWag, this->nPoints, this->nGen);}
	MapCM xiOf(const Eigen::VectorXd& x) const { return MapCM(x.data() + this->nPoints*this->nWag, this->nPoints, this->nGen);}
	MapM psiOf(Eigen::VectorXd& x) const { return MapM(x.data() + this->nPoints*(this->nWag+this->nGen), this->nPoints, 3);}
	MapCM psiOf(const Eigen::VectorXd& x) const { return MapCM(x.data() + this->nPoints*(this->nWag+this->nGen), this->nPoints, 3);}

	double span = 10.0;

	double cRef = 1.0;
	double s = 0.0;

	double u = 100.0; //Default airspeed
	double du = 0.0;
	double rAoA = 0.0; //Default root AoA
	double rho = 1.225;

	double uPrev = 100.00;
	double sPrev = 0.0;

	double q = 6125;
	
	Eigen::VectorXd z;
	Eigen::VectorXd y;

	Eigen::VectorXd c; 			//Chord
	Eigen::VectorXd cc;			//C_ref normalized by 
	Eigen::MatrixX2d v;			//Inputs
	Eigen::VectorXd cAlpha;		//Lift coefficient slope
	Eigen::VectorXd c0;			//Zero angle lift
	Eigen::MatrixXd aWag;		//Wagner A constants
	Eigen::MatrixXd lamWag;		//Wagner B constants
	Eigen::VectorXd dr;			//Rotation axis semichord normalized coordinate
	Eigen::MatrixXd eta;		//Influence matrix
	Eigen::MatrixXd aGen;		//Gennaretti A constants
	Eigen::MatrixXd lamGen;		//Gennaretti B constants
	
	Eigen::VectorXd dWy;
	Eigen::VectorXd lc;
	Eigen::VectorXd l;

    bool isConnected = false;
    bool isSet = false;
    bool isConfigured = false;

    DataType dType = DataType::DOB;
    interpType iType;

	const double* airspeed = nullptr;
	const double* rootAoA = nullptr;

    const SpanwiseVec<double>* realPitchAccel = nullptr;
    const SpanwiseVec<double>* realPlungeAccel = nullptr;
    const SpanwiseVec<airfoil>* realGeometry = nullptr;

    SpanwiseVec<double>* realLift = nullptr;
    double* realTotalLift = nullptr;
    double* realRollingMoment = nullptr;

};

inline Eigen::VectorXd cosineCoords(size_t nPts){ //Includes tips
	Eigen::VectorXd result(nPts);
	for(size_t i = 0; i<nPts - 1; i++){
		result(i) = (i*M_PI)/(nPts - 1);
	}
	result(nPts - 1) = M_PI;
	return result;
}

inline Eigen::VectorXd vecToEigen(const std::vector<double>& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	Eigen::VectorXd result(vec.size());
	for(size_t i = 0; i<vec.size(); i++){
		result(i) = vec[i];
	}
	return result;
}

inline Eigen::VectorXd airfoilVecToEigenChord(const SpanwiseVec<airfoil>& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	Eigen::VectorXd result(vec.size());
	for(size_t i = 0; i<vec.size(); i++){
		result(i) = vec[i].getChord();
	}
	return result;
}

inline Eigen::VectorXd airfoilVecToEigenCAlpha(const SpanwiseVec<airfoil>& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	Eigen::VectorXd result(vec.size());
	for(size_t i = 0; i<vec.size(); i++){
		result(i) = vec[i].getCAlpha();
	}
	return result;
}

inline Eigen::VectorXd airfoilVecToEigenC0(const SpanwiseVec<airfoil>& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	Eigen::VectorXd result(vec.size());
	for(size_t i = 0; i<vec.size(); i++){
		result(i) = vec[i].getC0();
	}
	return result;
}

inline Eigen::VectorXd airfoilVecToEigenDr(const SpanwiseVec<airfoil>& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	Eigen::VectorXd result(vec.size());
	for(size_t i = 0; i<vec.size(); i++){
		result(i) = vec[i].getAttatchmentPoint().x;
	}
	return result;
}

inline std::vector<double> eigenToVec(const Eigen::VectorXd& vec, size_t nPts){
	if(nPts != vec.size()) throw std::runtime_error("Cannot change number of stations specified at this point");
	std::vector<double> result;
	result.reserve(nPts);
	for(size_t i = 0; i<vec.size(); i++){
		result.push_back(vec(i));
	}
	return result;
}



}

#endif
