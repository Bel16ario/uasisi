#include "uasisi/core/xfoilWrapper.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

extern "C" {
	#include "libxfoil.h"
}

constexpr double rad2deg = 180.0/M_PI;

namespace uasisi{

Xfoil::Xfoil(const XfoilConfig& config){
	this->impl_=std::make_unique<Impl>();
	xfoil_options_type opts{config.nCrit, config.xTripTop, config.xTripBot, config.viscous, true, config.maxIter, config.vAccel};
	this->impl_->geom = {config.nPanels, 1.0, 0.15, 0.2, 1.0, 1.0, 1.0, 1.0};
	xfoil_init(&this->impl_->xdg);
	xfoil_defaults(&this->impl_->xdg, &opts);
	xfoil_set_paneling(&this->impl_->xdg, &impl_->geom);
}

Xfoil::~Xfoil(){
	if(this->impl_) xfoil_cleanup(&impl_->xdg);
}

Xfoil::Xfoil(Xfoil&&) noexcept = default;

Xfoil& Xfoil::operator=(Xfoil&& o) noexcept{
	if(this != &o){
		if(this->impl_) xfoil_cleanup(&impl_->xdg);
		this->impl_ = std::move(o.impl_);
	}
	return *this;
}

void Xfoil::setAirfoil(const std::vector<double>& x, const std::vector<double>& z){
	if(x.size() != z.size() || x.size() < 10) throw std::runtime_error("Xfoil: Invalid coordinates");
	std::vector<double> xs(x);
	std::vector<double> zs(z);
	int n = static_cast<int>(xs.size());
	int statusCode = 0;
	xfoil_set_buffer_airfoil(&this->impl_->xdg, xs.data(), zs.data(), &n);
	xfoil_smooth_paneling(&this->impl_->xdg, &statusCode);
	if(statusCode != 0) throw std::runtime_error("Xfoil: Paneling failed");
	xfoil_reinitialize_bl(&this->impl_->xdg);
	this->impl_->hasAirfoil = true;
}

void Xfoil::setAirfoil(const airfoil& foil){
	std::vector<double> x, z;
	for(size_t i = 0; i < foil.size(); i++){
		x.push_back(foil[i].x);
		z.push_back(foil[i].y);
	}
	this->setAirfoil(x,z);
}

void Xfoil::setFlow(double Re, double Ma){
	if(Re <= 0 || Ma < 0 || Ma >= 1) throw std::runtime_error("Xfoil: invalid flow conditions");
	xfoil_set_reynolds_number(&this->impl_->xdg, &Re);
	xfoil_set_mach_number(&this->impl_->xdg, &Ma);
	xfoil_reinitialize_bl(&this->impl_->xdg);
	this->impl_->hasFlow = true;
}

PolarPoint Xfoil::solve(double alpha){
	if(!this->impl_->hasAirfoil || !impl_->hasFlow) throw std::runtime_error("Xfoil: airfoil or flow conditions unset");
	double aDeg = alpha * rad2deg;
	double aOut;
	double cl;
	double cd;
	double cm;
	bool converged = false;
	int statusCode = 0;
	xfoil_specal(&this->impl_->xdg, &aDeg, &aOut, &cl, &cd, &cm, &converged, &statusCode);
	if(statusCode != 0) converged = false;
	if(!converged) xfoil_reinitialize_bl(&this->impl_->xdg); //Clear on failures
	return {alpha, cl, cd, cm, converged};
}

std::vector<PolarPoint> Xfoil::polar(double alphaMin, double alphaMax, double dAlpha){
	if(!(dAlpha > 0) || !(alphaMin <= 0 && alphaMax >= 0)) throw std::runtime_error("Xfoil: invalid angle grid");
	std::vector<PolarPoint> result;
	xfoil_reinitialize_bl(&this->impl_->xdg);
	for(int i = 0; i*dAlpha <= alphaMax + 1e-12; i++){
		result.push_back(this->solve(i*dAlpha));
	}
	xfoil_reinitialize_bl(&this->impl_->xdg);
	for(int i = 1; -i*dAlpha >= alphaMin - 1e-12; i++){
		result.push_back(this->solve(-i*dAlpha));
	}
	std::sort(result.begin(), result.end(), [](const PolarPoint& a, const PolarPoint& b){return a.alpha < b.alpha;});
	return result;
}

std::tuple<std::vector<double>, std::vector<double>> Xfoil::polarLift(double alphaMin, double alphaMax, double dAlpha){
	if(!(dAlpha > 0) || !(alphaMin <= 0 && alphaMax >= 0)) throw std::runtime_error("Xfoil: invalid angle grid");
	std::vector<double> resultAlpha;
	std::vector<double> resultCl;
    for(const PolarPoint& pt : this->polar(alphaMin, alphaMax, dAlpha)){
        if(pt.converged) resultAlpha.push_back(pt.alpha); resultCl.push_back(pt.cl);
    }
    return {resultAlpha, resultCl};
}




}
