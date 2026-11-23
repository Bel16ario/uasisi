#include "uasisi/utils/linearPolarHelper.hpp"

#include <limits>
#include <vector>
#include <stdexcept>

namespace uasisi{

struct LeastSquares{
	double n = 0.0;
	double sx = 0.0;
	double sy = 0.0;
	double sxx = 0.0;
	double sxy = 0.0;
	void add(double x, double y){
		n++;
		sx += x;
		sy += y;
		sxx += x*x;
		sxy += x*y;
	}
	double m() const {return (n*sxy - sx*sy) / (n*sxx - sx*sx);}
	double b() const {return (sy - m()*sx) / n;}
};

double localSlope(const std::vector<double>& alpha, const std::vector<double>& cl, size_t alpha0, const PolarAnalysisConfig& config){
	LeastSquares ls;
}

AirfoilLinearParams getLinearParamsFromPolar(const std::vector<double>& alpha, const std::vector<double>& cl, const PolarAnalysisConfig& config){

    if(alpha.size() != cl.size()) throw std::runtime_error("Cl and Alpha vectors do not match");
	if(alpha.size() < 5) throw std::runtime_error("Polar too short");
	for(size_t i = 1; i < alpha.size(); i++){
		if(alpha[i] < alpha[i-1]) throw std::runtime_error("Alpha vector must be monotonic and increasing");
	}
	if(config.frac < 0 || config.frac > 1 || config.maxGap <= 0 || config.stallDrop <= 0 || config.window <= 0){
		throw std::runtime_error("Invalid options");
	}

    
    double alpha0L;
    size_t alpha0LIndex;
    double dCl = std::numeric_limits<double>::max();
    double dClNew;

    for(size_t i = 0; i < alpha.size(); i++){
        dClNew = std::abs(cl[i]);
        if(dClNew < dCl){
            alpha0L = alpha[i];
            alpha0LIndex = i;
            dCl = dClNew;
        }
    }


}

}
