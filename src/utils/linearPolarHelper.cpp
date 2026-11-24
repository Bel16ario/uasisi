#include "uasisi/utils/linearPolarHelper.hpp"

#include <algorithm>
#include <cmath>
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
    if(alpha.size() != cl.size()) throw std::runtime_error("Cl and Alpha vectors do not match");
	LeastSquares ls;
    for(size_t i = alpha0; ; i--){
        if(alpha[alpha0] - alpha[i] > 0.5*config.window) break;
        ls.add(alpha[i], cl[i]);
        if(i == 0 || alpha[i] - alpha[i-1] > config.maxGap) break;
    }
    for(size_t i = alpha0 + 1; i < alpha.size(); i++){
        if(alpha[i] - alpha[alpha0] > 0.5*config.window || alpha[i] - alpha[i-1] > config.maxGap) break;
        ls.add(alpha[i], cl[i]);
    }
    if(ls.n < 3) throw std::runtime_error("Polar too short");
    return ls.m();
}

bool gap(const std::vector<double>& alpha, const PolarAnalysisConfig& config, size_t i){
    if(alpha.size() < i+2) throw std::runtime_error("Vector too short");
    return alpha[i+1] - alpha[i] > config.maxGap;
}

AirfoilLinearParams getLinearParamsFromPolar(const std::vector<double>& alpha, const std::vector<double>& cl, const PolarAnalysisConfig& config){

    if(alpha.size() != cl.size()) throw std::runtime_error("Cl and Alpha vectors do not match");
	if(alpha.size() < 5) throw std::runtime_error("Polar too short");
	for(size_t i = 1; i < alpha.size(); i++){
		if(alpha[i] <= alpha[i-1]) throw std::runtime_error("Alpha vector must be monotonic and increasing");
	}
	if(config.frac < 0 || config.frac > 1 || config.maxGap <= 0 || config.stallDrop <= 0 || config.window <= 0){
		throw std::runtime_error("Invalid options");
	}
    
    size_t alpha0LIndex = 0;
    double dCl = std::numeric_limits<double>::max();
    double dClNew;

    for(size_t i = 0; i < alpha.size(); i++){
        dClNew = std::abs(cl[i]);
        if(dClNew < dCl){
            alpha0LIndex = i;
            dCl = dClNew;
        }
    }

    size_t stallPos = alpha0LIndex;
    size_t stallNeg = alpha0LIndex;
    bool hasStalledPos = false;
    bool hasStalledNeg = false;

    for(size_t i = alpha0LIndex + 1; i < alpha.size(); i++){
        if(gap(alpha, config, i - 1)){hasStalledPos = true; break;}
        if(cl[i] > cl[stallPos]) stallPos = i;
        else if(cl[stallPos] - cl[i] > config.stallDrop){hasStalledPos = true; break;}
    }
    if(alpha0LIndex > 0){
        for(size_t i = alpha0LIndex - 1 ; i > 0; i--){
            if(gap(alpha, config, i)){hasStalledNeg = true; break;}
            if(cl[i] < cl[stallNeg]) stallNeg = i;
            else if(cl[i] - cl[stallNeg] > config.stallDrop){hasStalledNeg = true; break;}
        }
    }

    double ref = localSlope(alpha, cl, alpha0LIndex, config);
    if(ref <= 0) throw std::runtime_error("Slope at the zero lift angle is not positive");
    size_t pos = stallPos;
    size_t neg = stallNeg;
    bool hasReachedPos = hasStalledPos;
    bool hasReachedNeg = hasStalledNeg;
    for(size_t i = alpha0LIndex + 1; i <= stallPos; i++){
        if(gap(alpha, config, i-1) || !(localSlope(alpha, cl, i, config) >= config.frac*ref)){
            pos = i - 1;
            hasReachedPos = true;
            break;
        }
    }
    if(alpha0LIndex > 0){
        for(size_t i = alpha0LIndex - 1; i > stallNeg; i--){
            if(gap(alpha, config, i) || !(localSlope(alpha, cl, i, config) >= config.frac*ref)){
                neg = i + 1;
                hasReachedNeg = true;
                break;
            }
        }
    }

    LeastSquares ls;
    for(size_t i = neg; i <= pos; i++) ls.add(alpha[i], cl[i]);
    bool converged = hasStalledNeg && hasStalledPos && hasReachedPos && hasReachedNeg;
    if(ls.n >= 3) return{ls.m(), ls.b(), alpha[neg], alpha[pos], alpha[stallPos], alpha[stallNeg], converged};
    return{NAN, NAN, alpha[neg], alpha[pos], alpha[stallPos], alpha[stallNeg], converged};

}

AirfoilLinearParams getIntersectionLinearLimits(const std::vector<AirfoilLinearParams>& foils){
    if(foils.empty()) throw std::runtime_error("No polars provided");
    AirfoilLinearParams result;
    result.cAlpha = 0.0;
    result.c0 = 0.0;
    result.alphaMinLinear = -INFINITY;
    result.alphaMaxLinear = INFINITY;
    result.alphaNegStall = -INFINITY;
    result.alphaPosStall = INFINITY;
    result.converged = true;
    double sumM = 0.0;
    double sumB = 0.0;
    size_t nValid = 0.0;
    for(const AirfoilLinearParams& p : foils){
        result.alphaMinLinear = std::max(result.alphaMinLinear, p.alphaMinLinear);
        result.alphaMaxLinear = std::min(result.alphaMaxLinear, p.alphaMaxLinear);
        result.alphaNegStall = std::max(result.alphaNegStall, p.alphaNegStall);
        result.alphaPosStall = std::min(result.alphaPosStall, p.alphaPosStall);
        result.converged = result.converged && p.converged;

        if(p.converged){sumM += p.cAlpha; sumB += p.c0; nValid++;}
    }
    if(nValid) {result.cAlpha = sumM/nValid; result.c0 = sumB/nValid;}
    return result;
}

}
