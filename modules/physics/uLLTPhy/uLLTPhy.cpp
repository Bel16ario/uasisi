//TODO: Implement obtainPolars() with xfoil and also add number of points and force angular distribution.

#include "uLLTPhy.hpp"
#include "uasisi/core/types.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uasisi{

ULLTPhy::ULLTPhy(){
std::cout << "Unsteady LLT Physics module created\n";
}

void ULLTPhy::configure(const ULLTConfig& config){
	if(config.nPts < 5 || config.nPts % 2 == 0){
		throw std::runtime_error("An odd number of at least 5 points is needed");
	}
	if(config.span <= 0){
		throw std::runtime_error("A positive finite span must be specified");
	}
	if(config.rho <= 0){
		throw std::runtime_error("Invalid density");
	}
	if(config.cRef <= 0){
		throw std::runtime_error("A positive reference chord must be specified");
	}
	if(config.psiInit.rows() != config.nPts || config.chiInit.rows() != config.nPts || config.xiInit.rows() != config.nPts){
		throw std::runtime_error("Size mismatch");
	}
	if(config.lamGen.rows() != config.nPts || config.lamWag.rows() != config.nPts || config.aGen.rows() != config.nPts || config.aWag.rows() != config.nPts){
		throw std::runtime_error("Size mismatch");
	}
	if(config.chiInit.cols() != config.aWag.cols() || config.chiInit.cols() != config.lamWag.cols()){
		throw std::runtime_error("Size mismatch");
	}
	if(config.xiInit.cols() != config.aGen.cols() || config.xiInit.cols() != config.lamGen.cols()){
		throw std::runtime_error("Size mismatch");
	}


	this->nPoints = config.nPts;
	this->z = cosineCoords(config.nPts);
	this->y = (-0.5*config.span*this->z.array().cos()).matrix();

	this->nWag = config.aWag.cols();
	this->nGen = config.aGen.cols();

	this->X.resize(this->nPoints*(this->nGen+this->nWag + 3));

	chiOf(X) = config.chiInit;
	xiOf(X) = config.xiInit;
	psiOf(X) = config.psiInit;
	this->integrator = config.integrator;

	this->span = config.span;
	this->rho = config.rho;
	this->cRef = config.cRef;
	this->aWag = config.aWag;
	this->lamWag = config.lamWag;
	this->aGen = config.aGen;
	this->lamGen = config.lamGen;
	this->du = config.du;
	this->v.setZero(config.nPts, 2);
	this->l.resize(config.nPts);
	this->isConfigured = true;
}

void ULLTPhy::init(){

    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    } 
    if(!this->isConfigured){
        throw std::runtime_error("Module has not been configured yet");
    }
    if(!this->isConnected){
        throw std::runtime_error("Module connections have not been validated yet");
    }
    if((this->realGeometry || this->realPitchAccel || this->realPlungeAccel || this->airspeed || this->rootAoA) && (this->realLift || this->realTotalLift || this->realRollingMoment)){ // Input and output connected

		if(this->realGeometry){
			this->c = airfoilVecToEigenChord(*this->realGeometry, this->nPoints);
			this->cc = this->cRef / this->c.array();
			Eigen::Index end = this->nPoints - 1;
			this->cc(0) = 1;
			this->cc(end) = 1;
			this->cAlpha = airfoilVecToEigenCAlpha(*this->realGeometry, this->nPoints);
			this->c0 = airfoilVecToEigenC0(*this->realGeometry, this->nPoints);
			this->dr = airfoilVecToEigenDr(*this->realGeometry, this->nPoints);
		}
		if(this->airspeed) this->u = *this->airspeed;
		if(this->rootAoA) this->rAoA = *this->rootAoA;
		Eigen::VectorXd k = this->c.array()/(2*this->u);
		if(this->realPitchAccel) this->v.col(0) = k.array()*k.array()*vecToEigen(this->realPitchAccel->SWData(), this->nPoints).array();
		if(this->realPlungeAccel) this->v.col(1) = (2/this->c.array())*k.array()*k.array()*vecToEigen(this->realPlungeAccel->SWData(), this->nPoints).array();

		this->q = this->u * this->u * this->rho * 0.5;
		this->calcEta();
		this->calcLift();
		if(this->realLift) this->realLift->set(eigenToVec(this->z, this->nPoints), eigenToVec(this->l, this->nPoints));
		if(this->realTotalLift) *this->realTotalLift = trapz(this->y, this->l);
		if(this->realRollingMoment) *this->realRollingMoment = trapzProduct(this->y, this->l, this->y);

    } else if(this->realGeometry || this->realPitchAccel || this->realPlungeAccel || this->airspeed || this->rootAoA){ //Only input connected
        std::cout << "WARNING: no output connected. Module will IDLE. Connect monitor to force computation\n";
    } else if(this->realLift || this->realTotalLift || this->realRollingMoment){ //Only output connected
        throw std::runtime_error("No input connected. Cannot compute");
    } else { //No input or output
        std::cout << "WARNING: No input or output. Module will IDLE\n";
    }
    this->isSet = true;

}

std::vector<SignalInfo> ULLTPhy::declareSignals(){
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(8);
    reqSignals.push_back(SignalInfo("rootAoA", DataType::SCA, SignalType::IN));
    reqSignals.push_back(SignalInfo("airspeed", DataType::SCA, SignalType::IN));
    reqSignals.push_back(SignalInfo("realPitchAccel", DataType::DOB, SignalType::IN));
    reqSignals.push_back(SignalInfo("realPlungeAccel", DataType::DOB, SignalType::IN));
    reqSignals.push_back(SignalInfo("realGeometry", DataType::AIR, SignalType::IN));
    reqSignals.push_back(SignalInfo("realLift", DataType::DOB, SignalType::OUT));
    reqSignals.push_back(SignalInfo("realTotalLift", DataType::SCA, SignalType::OUT));
    reqSignals.push_back(SignalInfo("realRollingMoment", DataType::SCA, SignalType::OUT));
    return reqSignals;
}

void ULLTPhy::validateConnections(){

    size_t inputCount = bool(this->realPitchAccel) + bool(this->realPlungeAccel) + bool(this->realGeometry) + bool(this->airspeed) + bool(this->rootAoA);
    size_t outputCount = bool(this->realLift) + bool(this->realTotalLift) + bool(this->realRollingMoment);
    if(this->inputs.size() != inputCount || this->outputs.size() != outputCount){
        throw std::runtime_error("Unexpected error with signal ledgers");
    }
    if(inputCount != 5){
        std::cout << "WARNING: some inputs might be unconnected\n";
    }
    if(outputCount != 3){
        std::cout << "WARNING: some outputs might be unconnected\n";
    }
    if(this->airspeed){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "airspeed" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->rootAoA){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "rootAoA" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realPitchAccel){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "realPitchAccel" && s.dataType() == DataType::DOB;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realPlungeAccel){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "realPlungeAccel" && s.dataType() == DataType::DOB;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realGeometry){
        bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [](const SignalInfo& s){return s.name() == "realGeometry" && s.dataType() == DataType::AIR;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realLift){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realLift" && s.dataType() == DataType::DOB;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realTotalLift){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realTotalLift" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    if(this->realRollingMoment){
        bool cond = std::any_of(this->outputs.begin(), this->outputs.end(), [](const SignalInfo& s){return s.name() == "realRollingMoment" && s.dataType() == DataType::SCA;});
        if(!cond){
            throw std::runtime_error("Validation failed");
        }
    }
    this->isConnected = true;

}

void ULLTPhy::connectOutputDouble(const std::string& nName, SpanwiseVec<double>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nName == "realLift"){
        this->realLift = x;
        this->outputs.push_back(SignalInfo("realLift", DataType::DOB, SignalType::OUT));
        std::cout << "Connected output: " << nName << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + nName);
    }

}

void ULLTPhy::connectOutputScalar(const std::string& nName, double* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nName == "realTotalLift"){
        this->realTotalLift = x;
        this->outputs.push_back(SignalInfo("realTotalLift", DataType::SCA, SignalType::OUT));
        std::cout << "Connected output: " << nName << std::endl;
    } else if(nName == "realRollingMoment") {
        this->realRollingMoment = x;
        this->outputs.push_back(SignalInfo("realRollingMoment", DataType::SCA, SignalType::OUT));
        std::cout << "Connected output: " << nName << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + nName);
    }

}

void ULLTPhy::connectInputScalar(const std::string& nName, const double* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nName == "airspeed"){
        this->airspeed = x;
        this->inputs.push_back(SignalInfo("airspeed", DataType::SCA, SignalType::IN));
        std::cout << "Connected input: " << nName << std::endl;
    } else if(nName == "rootAoA"){
        this->rootAoA = x;
        this->inputs.push_back(SignalInfo("rootAoA", DataType::SCA, SignalType::IN));
        std::cout << "Connected input: " << nName << std::endl;
	}else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + nName);
    }

}

void ULLTPhy::connectInputDouble(const std::string& nName, const SpanwiseVec<double>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nName == "realPitchAccel"){
        this->realPitchAccel = x;
        this->inputs.push_back(SignalInfo("realPitchAccel", DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << nName << std::endl;
    } else if(nName == "realPlungeAccel") {
        this->realPlungeAccel = x;
        this->inputs.push_back(SignalInfo("realPlungeAccel", DataType::DOB, SignalType::IN));
        std::cout << "Connected input: " << nName << std::endl;
	} else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + nName);
    }

}

void ULLTPhy::connectInputAirfoil(const std::string& nName, const SpanwiseVec<airfoil>* x){

    if(this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if(nName == "realGeometry"){
        this->realGeometry = x;
        this->inputs.push_back(SignalInfo("realGeometry", DataType::AIR, SignalType::IN));
        std::cout << "Connected input: " << nName << std::endl;
    } else {
        throw std::runtime_error("Unrecognised signal. Check that the name is correct: " + nName);
    }

}

void ULLTPhy::step(double t, double dt){
    if(!this->isSet || !this->isConfigured || !this->isConnected){
        throw std::runtime_error("Module is not initialized/configured/validated");
    }
	this->uPrev = this->u;
	this->sPrev = this->s;
    
    if((this->realGeometry || this->realPitchAccel || this->realPlungeAccel || this->airspeed || this->rootAoA) && (this->realLift || this->realTotalLift || this->realRollingMoment)){ // Input and output connected
		if(this->airspeed) this->u = *this->airspeed;
		if(this->rootAoA) this->rAoA = *this->rootAoA;
		this->s += (this->u + this->uPrev)*dt/this->cRef;
		this->calcDu();
		Eigen::VectorXd k = this->c.array()/(2*this->u);
		if(this->realPitchAccel) this->v.col(0) = k.array()*k.array()*vecToEigen(this->realPitchAccel->SWData(), this->nPoints).array();
		if(this->realPlungeAccel) this->v.col(1) = (2/this->c.array())*k.array()*k.array()*vecToEigen(this->realPlungeAccel->SWData(), this->nPoints).array();

		this->q = this->u * this->u * this->rho * 0.5;
		integrate(this->s - this->sPrev);
		this->calcLift();
		if(this->realLift) this->realLift->set(eigenToVec(this->z, this->nPoints), eigenToVec(this->l, this->nPoints));
		if(this->realTotalLift) *this->realTotalLift = trapz(this->y, this->l);
		if(this->realRollingMoment) *this->realRollingMoment = trapzProduct(this->y, this->l, this->y);
    } else if(this->realGeometry || this->realPitchAccel || this->realPlungeAccel || this->airspeed || this->rootAoA){ //Only input connected
        std::cout << "WARNING: no output connected. Module will IDLE. Connect monitor to force computation\n";
    } else if(this->realLift || this->realTotalLift || this->realRollingMoment){ //Only output connected
        throw std::runtime_error("No input connected. Cannot compute");
    } else { //No input or output
        std::cout << "WARNING: No input or output. Module will IDLE\n";
    }

}

void ULLTPhy::calcEta(){
	if(!this->isConfigured) throw std::runtime_error("Module is not configured yet");
	this->eta.resize(this->nPoints, this->nPoints);
	this->eta.setZero();
	for(size_t i = 1; i < this->nPoints - 1; i++){ //Tips are not counted here
		for(size_t j = 1; j < this->nPoints - 1; j++){
			if(i == j) this->eta(i,j) = (nPoints-1)/(4*sin(z[i]));
			else if((j-i) % 2 == 0) this->eta(i,j) = 0;
			else this->eta(i,j) = ( (sin(z[j])) / ((cos(z[j]) - cos(z[i]))*(cos(z[j]) - cos(z[i]))) )/(nPoints - 1);
		}
	}
}

void ULLTPhy::calcLift(){
	if(!this->isConfigured) throw std::runtime_error("Module is not configured yet");
	Eigen::VectorXd wy, aE;
	MapM psi = psiOf(this->X);
	algebraic(this->X, wy, aE, this->lc);
	this->calcDwy(wy);
	Eigen::VectorXd f = this->du/(this->u*this->cc.array());//technically a frequency when dimensional
	Eigen::VectorXd lnc = M_PI*(this->v.col(1).array() + psi.col(1).array() + this->dr.array()*this->v.col(0).array() - this->dWy.array() + f.array()*(psi.col(0).array() - wy.array())).matrix();
	this->l = (this->q*this->c.array()*(this->lc.array() + lnc.array())).matrix();

}

void ULLTPhy::algebraic(const Eigen::VectorXd& x, Eigen::VectorXd& wy, Eigen::VectorXd& alphaE, Eigen::VectorXd& lcirc){
	MapCM chi = chiOf(x);
	MapCM xi = xiOf(x);
	MapCM psi = psiOf(x);
	Eigen::VectorXd g = -(this->aGen.array()*this->lamGen.array()*xi.array()).rowwise().sum() / (this->rho*this->span*this->u*this->u); 
	wy = 2*this->eta.diagonal().cwiseProduct(g) - this->eta*g;
	const Eigen::ArrayXd wcin = -(psi.col(2).array() + (0.5 + this->dr.array())*psi.col(1).array());
	alphaE = (psi.col(0).array() + this->rAoA - wy.array() - wcin).matrix();
	lcirc = (0.5*(this->cAlpha.array()*alphaE.array() + this->c0.array()) - (this->aWag.array()*this->lamWag.array()*chi.array()).rowwise().sum()).matrix();
	Eigen::Index end = this->nPoints - 1;
	lcirc(0) = 0;
	lcirc(end) = 0;
}


void ULLTPhy::rates(const Eigen::VectorXd& x, Eigen::VectorXd& dx){
	Eigen::VectorXd wy, aE, lcirc;
	algebraic(x, wy, aE, lcirc);
	dx.resize(x.size());
	MapCM chi = chiOf(x);
	MapCM xi = xiOf(x);
	MapCM psi = psiOf(x);
	MapM dChi = chiOf(dx);
	MapM dXi = xiOf(dx);
	MapM dPsi = psiOf(dx);

	Eigen::VectorXd f = this->du/(this->u*this->cc.array());
	dChi.array() = (this->lamWag.array()*chi.array()).colwise() + (this->cAlpha.array()*aE.array() + this->c0.array());
	dXi.array() = (this->lamGen.array()*xi.array()).colwise() + this->q*this->c.array()*lcirc.array();
	dPsi.col(0)  = psi.col(1);
	dPsi.col(1) = this->v.col(0).array() - f.array()*psi.col(1).array();
	dPsi.col(2) = this->v.col(1).array() - f.array()*psi.col(2).array();

	Eigen::Index end = this->nPoints - 1;
	dChi.row(0).setZero();
	dChi.row(end).setZero();
	dXi.row(0).setZero();
	dXi.row(end).setZero();

	dChi.array().colwise() *= this->cc.array();
	dXi.array().colwise() *= this->cc.array();
	dPsi.array().colwise() *= this->cc.array();
	
}

void ULLTPhy::integrate(double ds){
	switch(this->integrator){
		case IntegratorScheme::Euler:
			stepEuler(ds); break;
		case IntegratorScheme::ExpEuler:
			stepExpEuler(ds); break;
		case IntegratorScheme::ZOH:
			stepZOH(ds); break;
		case IntegratorScheme::Heun:
			stepHeun(ds); break;
		case IntegratorScheme::RK4:
			stepRK4(ds); break;
		case IntegratorScheme::RFK45:
			stepRFK45(ds); break;
	}
}


void ULLTPhy::calcDu(){
	if(!this->isConfigured) throw std::runtime_error("Module is not configured yet");
	if(!this->isSet) return; //Do not fire on init. Initial du provided
	if(this->s - this->sPrev < 0) throw std::runtime_error("Negative velocity encountered");
	if(this->s - this->sPrev > 0) this->du = (this->u - this->uPrev)/(this->s - this->sPrev);
}

void ULLTPhy::calcDwy(const Eigen::VectorXd& nWy){
	if(!this->isConfigured) throw std::runtime_error("Module is not configured yet");
	if(nWy.rows() != this->nPoints) throw std::runtime_error("Size mismatch");
	MapM xi = xiOf(this->X);
	Eigen::VectorXd sum = (this->aGen.array()*this->lamGen.array()*xi.array()).rowwise().sum().matrix();
	Eigen::VectorXd dSumRef = (this->cc.array()*(((this->aGen.array()*this->lamGen.array()*this->lamGen.array()*xi.array()).rowwise().sum() + (this->aGen.array()*this->lamGen.array()).rowwise().sum()*this->lc.array()*this->q*this->c.array()))).matrix();
	Eigen::VectorXd dGRef = (-dSumRef.array()/(this->rho*this->span*this->u*this->u) + (2*this->du*sum.array())/(this->rho*this->span*this->u*this->u*this->u)).matrix();
	this->dWy = ((2*this->eta.diagonal().array()*dGRef.array() - (this->eta*dGRef).array())/this->cc.array()).matrix();

}

void ULLTPhy::stepEuler(double ds){
	Eigen::VectorXd k1;
	this->rates(this->X, k1);
	this->X += ds*k1;
}

void ULLTPhy::stepExpEuler(double ds){
	this->stepEuler(ds);
}

void ULLTPhy::stepZOH(double ds){
	this->stepEuler(ds);
}

void ULLTPhy::stepHeun(double ds){
	this->stepEuler(ds);
}

void ULLTPhy::stepRK4(double ds){
	this->stepEuler(ds);
}

void ULLTPhy::stepRFK45(double ds){
	this->stepEuler(ds);
}


double trapz(const Eigen::VectorXd& x, const Eigen::VectorXd& y){

    if(x.size() != y.size() || x.size() < 4){
        throw std::runtime_error("vector size error");
    }
    double sum = 0.0;
    for(size_t i = 1; i < x.size(); i++){
        sum += 0.5 * (y(i) + y(i-1)) * (x(i) - x(i-1));
    }
    return sum;

}


    
double trapzProduct(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z){

    if(x.size() != y.size() || x.size() != z.size() || x.size() < 4){
        throw std::runtime_error("vector size error");
    }
    double sum = 0.0;
    for(size_t i = 1; i < x.size(); i++){
        sum += 0.5 * (y(i)*z(i) + y(i-1)*z(i-1)) * (x(i) - x(i-1));
    }
    return sum;

}

}

namespace{
static uasisi::ModuleRegistration<uasisi::ULLTPhy> registrationULLTPhy("ULLTPhy");
}
