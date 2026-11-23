#include "uasisi/io/runConfig.hpp"
#include "uasisi/core/xfoilWrapper.hpp"
#include "uasisi/core/types.hpp"

#include <limits>
#include <cmath>

namespace uasisi{

AirfoilLinearParams getLinearParams(const airfoil& foil, double h, double u, double thresh){

    FlightConditions flow;
    flow.setAltitude(h);
    double rho = flow.getDensity();
    double mu = flow.getViscosity();
    double T = flow.getTemperature();
    double chord = foil.getChord();
    double Re = (1/mu)*rho*u*chord;
    double Ma = std::sqrt(1.4*287.05*T);
    double alphaMin = -M_PI / 12.0;
    double alphaMax = M_PI / 12.0;
    double dAlpha = M_PI / 240;

    XfoilConfig xConfig; //Default settings
    Xfoil x(xConfig);
    x.setAirfoil(foil);
    x.setFlow(Re, Ma);
    auto [alpha, cl] = x.polarLift(alphaMin, alphaMax, dAlpha);

    return getLinearParamsFromPolar(alpha, cl, thresh);
}

AirfoilLinearParams getLinearParamsFromPolar(std::vector<double> alpha, std::vector<double> cl, double thresh){

    if(alpha.size() != cl.size()) throw std::runtime_error("Cl and Alpha vectors do not match");
    
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
