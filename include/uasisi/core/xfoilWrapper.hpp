#ifndef XFOIL_WRAPPER_HPP
#define XFOIL_WRAPPER_HPP

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

extern "C" {
	#include "libxfoil.h"
}

#include "uasisi/core/types.hpp"

namespace uasisi{

struct XfoilConfig{

	double nCrit = 9.0;
	double xTripTop = 1.0;
	double xTripBot = 1.0;

	bool viscous = true;
	int maxIter = 200;
	double vAccel = 0.01;
	int nPanels = 200;

};

struct PolarPoint{

	double alpha;
	double cl;
	double cd;
	double cm;
	bool converged;

};

class Xfoil{
public:
	
	explicit Xfoil(const XfoilConfig& config = {});
	~Xfoil();

	Xfoil(const Xfoil&) = delete;
	Xfoil& operator=(const Xfoil&) = delete;
	Xfoil(Xfoil&&) noexcept;
	Xfoil& operator=(Xfoil&&) noexcept;

	void setAirfoil(const airfoil& foil);
	void setAirfoil(const std::vector<double>& x, const std::vector<double>& z);
	void setFlow(double Re, double Ma);

	PolarPoint solve(double alpha);
	std::vector<PolarPoint> polar(double alphaMin, double alphaMax, double dAlpha);

	//Skips unconverged points. Returns alpha, cl
	std::tuple<std::vector<double>> polarLift(double alphaMin, double alphaMax, double dAlpha);

private:

	struct Impl{
		xfoil_data_group xdg;
		xfoil_geom_options_type geom;
		bool hasAirfoil = false;
		bool hasFlow = false;
	};

	std::unique_ptr<Impl> impl_;

};

inline double speedOfSound(double T){ return std::sqrt(1.4*287.05*T);}

}

#endif
