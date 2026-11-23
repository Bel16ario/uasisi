#ifndef RUNCONFIG_HPP
#define RUNCONFIG_HPP

#include <map>
#include <tuple>
#include <string>
#include <filesystem>
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "uasisi/core/types.hpp"
#include "uasisi/utils/linearPolarHelper.hpp"

namespace uasisi {

enum class PhysicsType {ULLT, ULLT0, ULLT1, ULLT2, ULLT3, DUST, SHARPy, OF};
enum class MotionType {TravellingWave, StandingWave, Uniform};
enum class SpacingType {Linear, Cosine};
enum class FitFavours{WAG, GEN, JOINT};

AirfoilLinearParams getLinearParams(const airfoil& foil, double h, double u, const PolarAnalysisConfig& config);

inline std::vector<double> interpWingParam(const SpanwiseVec<double>& param, const std::vector<double>& eta){
	return interpolate(param.coords(), param.SWData(), eta, getInterpType(interpType::LIN));
}

inline std::vector<airfoil> interpWingAirfoil(const SpanwiseVec<airfoil>& foils, const std::vector<double>& eta){
	return interpolate(foils.coords(), foils.SWData(), eta, getInterpType(interpType::LIN));
}


//Config = struct. Definition = class
struct StationValues{
	std::vector<double> eta;
	std::vector<double> beta;
	std::vector<double> chord;
	std::vector<double> cAlpha;
	std::vector<double> c0;
	std::vector<double> dR;
	std::vector<airfoil> foils;

};

class WingDefinition{

public:

	//Constructor checks that spanwise vector coordinates match. 
	WingDefinition(SpanwiseVec<double> chord, SpanwiseVec<airfoil> foils, double h, double u, double thresh);

	std::vector<double> getChord(size_t nStations, SpacingType spacing) const;
	std::vector<double> getCalpha(size_t nStations, SpacingType spacing) const;
	std::vector<double> getC0(size_t nStations, SpacingType spacing) const;
	std::vector<double> getDr(size_t nStations, SpacingType spacing) const;
	std::vector<double> getEta(size_t nStations, SpacingType spacing) const;
	std::vector<double> getBeta(size_t nStations, SpacingType spacing) const;
	double getMeanChord() const {return meanChord_;}
	const StationValues& getStationValues(size_t nStations, SpacingType spacing) const;
	
	double getAltitude() const {return h_;}
	double getMu() const {return mu_;}
	double getU() const {return u_;}
	double getRho() const {return rho_;}

	std::vector<AirfoilLinearParams> getLinearLimits(size_t nStations, SpacingType spacing, const std::vector<double>& uGrid) const;


private:

	std::vector<double> chord_;
	std::vector<double> coords_; //Original coordinates
	std::vector<airfoil> foils_; //Original airfoils
	double meanChord_;
	
	double h_;
	double u_;
	double mu_;
	double rho_;

	double thresh_;
	
	mutable std::map<std::pair<size_t, SpacingType>, StationValues> cache_;

};

struct VelocityConfig{

	double uAvg;
	double amp; //peak amplitude
	double k;
	double phase;

};

std::vector<double> velocityGrid(const VelocityConfig& uConf, size_t stepsPerCycle);

struct MotionConfig{

	MotionType motion;
	double alphaMin;
	double alphaMax;
	double periodsInSpan;
	double phase; //Offset from wingtip
	int direction;

};

struct TimeConfig{
	
	size_t nCycles;
	size_t stepsPerCycle;

};

struct LagSpec{
	size_t JWag;
	size_t JGen;
	double weight; //Wagner 1.0, Gennaretti = 0.0. Applies only to JOINT
	std::vector<FitFavours> weights; //WAG, GEN, JOINT
	std::vector<size_t> sharedPoles; //Indeces of the shared poles
	
	bool operator<(const LagSpec& o) const {
		return std::tie(JWag, JGen, weight, weights, sharedPoles)
			< std::tie(o.JWag, o.JGen, o.weight, o.weights, o.sharedPoles);
		}
};

struct LagSet{
	std::vector<double> aWag;
	std::vector<double> lamWag;

	std::vector<double> aGen;
	std::vector<double> lamGen;
};

using LagFitFunc = std::function<LagSet(const LagSpec&)>;

class LagLibrary{
public:

	explicit LagLibrary(LagFitFunc fit) : fit_(std::move(fit)) {}
	const LagSet& getLagSet(const LagSpec& s);
	size_t size() const {return cache_.size();}
private:

	LagFitFunc fit_;
	std::map<LagSpec, LagSet> cache_;
};

struct StationLags{
	Eigen::MatrixXd aWag;
	Eigen::MatrixXd lamWag;
	Eigen::MatrixXd aGen;
	Eigen::MatrixXd lamGen;
	std::vector<LagSpec> spec;
};

class LagConstants{
public:

	LagConstants(size_t JWag, size_t JGen, const std::vector<LagSpec>& spec);

	size_t getJWag() const {return JWag_;}
	size_t getJGen() const {return JGen_;}
	StationLags build(LagLibrary& lib) const;
	
private:
	size_t JWag_;
	size_t JGen_;
	std::vector<LagSpec> spec_;
};

struct SweepConfig{
	std::vector<double> k;
	std::vector<double> AR;
	std::optional<std::vector<double>> alphaMax;
	std::optional<std::vector<double>> alphaMin; //same size. Less than max
	std::optional<std::vector<size_t>> nStations; //less than ULLT max
	std::optional<std::vector<LagConstants>> lags;
};

struct ModuleSpec{
	PhysicsType physics;
	std::string name;
	YAML::Node raw;
};

struct RunConfig{
	std::string caseName;
	std::filesystem::path output;
	WingDefinition wing;
	VelocityConfig velocity;
	MotionConfig motion;
	TimeConfig time;
	SweepConfig sweep;
	std::vector<ModuleSpec> modules;
	std::string rawConfig;

};

RunConfig loadRunConfig(const std::string& filename);

struct RunCase{
	std::string id;
	double k;
	double AR;
	double alphaMin;
	double alphaMax;
	std::optional<size_t> nStations;
	std::optional<LagConstants> lags;
};

std::vector<RunCase> expandSweep(const RunConfig& cfg);

std::string toString(PhysicsType t);


}

#endif
