#include "uasisi/io/runConfig.hpp"
#include "uasisi/core/xfoilWrapper.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/utils/linearPolarHelper.hpp"

#include <limits>
#include <cmath>

namespace uasisi{


AirfoilLinearParams getLinearParams(const airfoil& foil, double h, double u, const PolarAnalysisConfig& config){

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

    return getLinearParamsFromPolar(alpha, cl, config);
}

}
