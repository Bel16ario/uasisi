#ifndef LINEAR_POLAR_HELPER_HPP
#define LINEAR_POLAR_HELPER_HPP

#include <vector>
#include <cmath>

namespace uasisi{

struct AirfoilLinearParams{
	double cAlpha;
	double c0;

	double alphaMinLinear;
	double alphaMaxLinear;

	double alphaPosStall;
	double alphaNegStall;

	bool converged;
};

struct PolarAnalysisConfig{
	double frac = 0.5;
	double window = M_PI / 180.0;
	double maxGap = 0.5*M_PI / 180.0;
	double stallDrop = 0.02;
};

double localSlope(const std::vector<double>& alpha, const std::vector<double>& cl, size_t alpha0, const PolarAnalysisConfig& config);

AirfoilLinearParams getLinearParamsFromPolar(const std::vector<double>& alpha, const std::vector<double>& cl, const PolarAnalysisConfig& config);

AirfoilLinearParams getIntesectionLinearLimits(const std::vector<AirfoilLinearParams>& params);


}

#endif
