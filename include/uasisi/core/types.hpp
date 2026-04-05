#ifndef TYPES_HPP
#define TYPES_HPP

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <set>
#include <type_traits>
#include <typeinfo>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <eigen3/Eigen/Dense>

namespace uasisi{

template<typename T, typename U, typename W>
std::vector<T> normalize(const std::vector<T>& vec, const U& a, const U& b, const W& c, const W& d);

template<typename T>
std::vector<T> interpolate(const std::vector<double>& z, const std::vector<T>& data, const std::vector<double>& zp, const gsl_interp_type* t);

template<typename T, typename U>
std::vector<T> extractMemberVector(const std::vector<U>& objects, T U::*member);

template<typename T, typename U>
std::vector<T> extractVariableVector(const std::vector<U>& objects, T (U::*getter)() const);

template<typename T>
std::vector<T> extractRowVector(const std::vector<std::vector<T>>& objects, size_t idx);

Eigen::Map<const Eigen::VectorXd> vecAsEigen(const std::vector<double>& x);

Eigen::Map<Eigen::VectorXd> vecAsEigenMutable(const std::vector<double>& x);

std::vector<double> EigenAsVec(const Eigen::VectorXd& x);

void updateVecWithEigen(const Eigen::VectorXd& x, std::vector<double>& vec);// non const reference is intended
    


struct point{
    double x;
    double y;
    double z;
    int idx;

    point operator+(const point& p2) const {
        return point{x + p2.x, y + p2.y, z + p2.z, idx};
    }

    point operator-(const point& p2) const {
        return point{x - p2.x, y - p2.y, z - p2.z, idx};
    }

    point operator*(double k) const {
        return point{x*k, y*k, z*k, idx};
    }


    point operator/(double k) const {
        return point{x/k, y/k, z/k, idx};
    }


    point operator-() const {
        return point{-x, -y, -z, idx};
    }

    point& operator+=(const point& p2) {
        x += p2.x;
        y += p2.y;
        z += p2.z;
        return *this;
    }

    point& operator-=(const point& p2) {
        x -= p2.x;
        y -= p2.y;
        z -= p2.z;
        return *this;
    }


    point& operator*=(double k) {
        x *= k;
        y *= k;
        z *= k;
        return *this;
    }


    point& operator/=(double k) {
        x /= k;
        y /= k;
        z /= k;
        return *this;
    }

    bool operator==(const point& p2) const {
        return x == p2.x && y == p2.y && z == p2.z;
    }

    bool operator!=(const point& p2) const {
        return !(*this == p2);
    }

    double magnitudeSquared(){
        return x*x + y*y + z*z;
    }

    double magnitude(){
        return std::sqrt(magnitudeSquared());
    }

    double norm(){
        return magnitude();
    }

    point normalized(){
        double mag = magnitude();
        if (mag == 0){ return point{0, 0, 0, idx};}
        return point{x / mag, y / mag, z / mag, idx};
    }

    void normalize(){
        double mag = magnitude();
        if(mag != 0){
            x = x / mag;
            y = y / mag;
            z = z / mag;
        }
    }

};

inline point operator*(double k, const point& p) {
    return p * k;
}

inline double dot(const point& p1, const point& p2){
    return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}

inline point cross(const point& p1, const point& p2){
    return point{(p1.y*p2.z - p1.z*p2.y), (p1.z*p2.x - p1.x*p2.z), (p1.x*p2.y - p1.y*p2.x), p1.idx};
}

class FlightConditions{//Should I put this class in modules.hpp or types.hpp so that other physics modules might use it?

    public:

    FlightConditions() = default;
    ~FlightConditions() = default;

    void compThetaAndP();
    void compDenAndVisc();
    void comp();

    void setVInf(const point& vNew); //set vInf AND alpha
    void setAltitude(const double& hNew); //Check hNew >= 0

    point getVelocity(){return this->vInf;}
    double getAltitude(){return this->altitude;}
    double getTemperature(){return this->theta0;}
    double getPressure(){return this->P0;}
    double getDensity(){return this->rho0;}
    double getViscosity(){return this->mu0;}
    double getAlpha(){return this->alpha;}

    private:

    point vInf = {.x = 100.0, .y = 0.0, .z = 0.0, .idx = 0}; //vector. it is assumed that rolling axis is fully horizontal (-1, 0, 0). the centerPos angle is with respect to this rolling axis. omega is with respect to this centerPos angle although here it is already passed as omega + centerPos from the actuator module.  m/s
    double altitude = 1000; //m
    double theta0;
    double P0;
    double rho0;
    double mu0;
    double alpha = 0.0; //Set from vInf

    bool vInfIsSet = false;
    bool altitudeIsSet = false;

};

enum class interpType {
    LIN, 
    CSP,
    AKI,
};
    
const gsl_interp_type* getInterpType(interpType t);

class airfoil{
public:

    airfoil() = default;
    airfoil(const std::string& name, const std::vector<point>& coords);

    size_t                          size() const {return points.size();}

    const std::vector<point>&       coords() const {return points;}
    const std::string&              name() const {return foilName;}
    
    const point&                    operator[](size_t i) const {return points[i];}

    void                            setName(const std::string& newName){this->foilName = newName; this->hasName = true;}

    double                          getChord() const {return this->chord;}
    void                            setChord(double c){this->chord = c;}
    point                           getAttatchmentPoint() const {return this->attatchmentPoint;}
    void                            setAttatchmentPoint(point p){this->attatchmentPoint = p;}
    void                            setPoints(const std::vector<point>& pVec){this->points = pVec;}
    point                           getPoint(int id) const;
    void                            pushPoint(point p){this->points.push_back(p);}

    int                             check(); //add later. check no repeated idx and points in correct order
    void                            heal(); //order points / maybe idx
    void                            generate(size_t n); //Generate from name (NACA)
    void                            updateName(); // from points, obtain the name (NACA)
    bool                            pointIsInSection(int id) const;

private:

    std::string                     foilName;
    std::vector<point>              points;
    double                          chord;
    point                           attatchmentPoint;

    bool                            hasName = false;
    bool                            isOk = false;

};

bool pointIsBounded(const std::vector<airfoil>& data, const std::vector<double>& z, double zp, int idx);

template<typename T>
class SpanwiseVec{
public:

    SpanwiseVec() = default;
    SpanwiseVec(const std::vector<double>& coords, const std::vector<T>& SWData);

    size_t                              size() const {return this->z.size();}

    const std::vector<double>&          coords() const {this->coordsHaveBeenRead = true; return this->z;} //Add coords have been read flag logic to eigen getters
    Eigen::Map<const Eigen::VectorXd>   coordsAsEigen() const {return uasisi::vecAsEigen(this->z);}
    Eigen::Map<Eigen::VectorXd>         coordsAsEigenMutable() {return uasisi::vecAsEigenMutable(this->z);}
    
    const std::vector<T>&               SWData() const {this->dataHasBeenRead = true; return this->data;}
    Eigen::Map<const Eigen::VectorXd>   SWDataAsEigen() const; //Only for double
    Eigen::Map<Eigen::VectorXd>         SWDataAsEigenMutable(); //more eigen converters could be added for variables of structs using the extractvariable function but seems unimportant for now. Ideally this class supports any data type but for now, only doubles, vectors of doubles and airfoils are supported.
    template<typename U = T, typename = std::enable_if_t<std::is_same_v<U, airfoil>>>
    Eigen::VectorXd                     SWDataMemberAsEigen(double (U::*getter)() const) const; //for airfoil. At the moment it is hardcoded
    Eigen::VectorXd                     SWDataRowAsEigen(size_t idx) const; //For vector
    
    bool                                hasNewData() const {return !this->dataHasBeenRead;} 
    bool                                hasNewCoords() const {return !this->coordsHaveBeenRead;} //I really should have made separate setters but now a lot of code uses the monolithic setter. At least it prevents size mismatches. 

    T&                                  operator[](size_t i){return this->data[i];} 
    const T&                            operator[](size_t i) const {return this->data[i];}

    double&                             z_at(size_t i) {return this->z[i];}
    const double&                       z_at(size_t i) const {return this->z[i];}

    void                                set(const std::vector<double>& zN, const std::vector<T>& data_N); //Would be cool to add set from eigen for both coords and data but I like the fect that there is only one set function. It forces the vector sizes to match so I will leave it like this at the cost of some efficiency.
    void                                setNoInterp(const std::vector<double>& zN, const std::vector<T>& dataN);
    void                                setZOrig(const std::vector<double>& zN);
    void                                setIsCoordNormalized(bool cond){this->isCoordNormalized = cond;}
    void                                setIsDataNormalized(bool cond){this->isDataNormalized = cond;}
    void                                setIsInterpolated(bool cond){this->isInterpolated = cond;}

    void                                updateBounds(){this->updateCoordBounds(); this->updateDataBounds();}
    void                                updateCoordBounds();
    void                                updateDataBounds();

    void                                setCoordBounds(double a, double b);
    void                                setScalarDataBounds(double a, double b);
    void                                setDataBounds(T a, T b);

    void                                setOrigCoordBounds(double a, double b);
    void                                setOrigScalarDataBounds(double a, double b);
    void                                setOrigDataBounds(T a, T b);
    
    void                                normalize(){this->normalizeCoords(); this->normalizeData();}

    void                                normalizeCoords();
    void                                normalizeCoordsFrom(double a, double b);
    void                                normalizeCoordsTo(double a, double b);
    void                                normalizeCoordsFromTo(double a, double b, double c, double d);

    void                                normalizeData();
    void                                normalizeDataFrom(T a, T b);
    void                                normalizeDataTo(T a, T b);
    void                                normalizeDataFromTo(T a, T b, T c, T d);

    void                                denormalize(){this->denormalizeCoords(); this->denormalizeData();}
    void                                denormalizeCoords();
    void                                denormalizeData();

    void                                interpolateData(std::vector<double> zp, interpType t);
    void                                uninterpolate();

    void                                restore(){this->denormalize(); this->uninterpolate();}

private:

    std::vector<double>                 z;
    std::vector<T>                      data;

    double                              z_min;
    double                              z_max;

    T                                   data_min;
    T                                   data_max;

    std::vector<double>                 z_noInterp;
    std::vector<double>                 z_orig;
    double                              z_min_orig;
    double                              z_max_orig;
    
    std::vector<T>                      data_noInterp;
    T                                   data_min_orig;
    T                                   data_max_orig;

    bool                                isCoordNormalized = false;
    bool                                isDataNormalized = false;
    bool                                isInterpolated = false;
    bool                                CoordBoundsSet = false;
    bool                                DataBoundsSet = false;
    bool                                isSet = false;

    mutable bool                        coordsHaveBeenRead = false;
    mutable bool                        dataHasBeenRead = false;

};

inline void FlightConditions::compThetaAndP(){
    if(this->altitude <= 11000.0){
        this->theta0 = 288.15 - 0.0065*this->altitude;
        this->P0 = 101325.0 / std::pow((this->theta0 / 288.15), (-9.80665 / 1.865825));
    } else{
        this->theta0 = 216.65;
        this->P0 = 101325.0 * 0.22336 * std::exp(-9.80665 * (this->altitude - 11000.0) / 62189.3825);
    }
}

inline void FlightConditions::compDenAndVisc(){
    this->compThetaAndP();
    this->rho0 = this->P0 / (this->theta0 * 287.05);
    this->mu0 = 1.716e-5 * std::pow(this->theta0 / 288.15, 1.5) * (288.15 + 110.4) / (this->theta0 + 110.4);
}

inline void FlightConditions::comp(){
    if(!this->altitudeIsSet || !this->vInfIsSet){
        std::cout << "WARNING: Flight conditions not set. Using defaults\n";
    }
    this->compDenAndVisc();
}

inline void FlightConditions::setAltitude(const double& hNew){
    if(hNew < 0.0 || hNew > 20000.0){
        throw std::runtime_error("Invalid altitude. Must be between 0 and 20K meters");
    }
    this->altitude = hNew;
    this->altitudeIsSet = true;
}

inline void FlightConditions::setVInf(const point& vNew){
    this->vInf = vNew;
    this->alpha = std::atan(this->vInf.y/this->vInf.x);
    this->vInfIsSet = true;
}

inline point airfoil::getPoint(int id) const {
    for(const point& p : this->points){
        if(p.idx == id) return p;
    }
    throw std::runtime_error("Point not in section");
}

inline bool airfoil::pointIsInSection(int id) const {
    for(const point& p : this->points){
        if(p.idx == id) return true;
    }
    return false;
}

inline void airfoil::generate(size_t n){ //generates double the amount of points
    if(!this->hasName) throw std::runtime_error("A name is required to generate points");
    std::string foil = this->name();
    if(!(foil.length() == 8 || foil.length() == 9) || foil.substr(0, 4) != "NACA") throw std::runtime_error("Only 4 and 5 digit NACA foils supported in the NACAXXXX or NACAXXXXX format");
    float maxCamber, maxCamberPos, thickness, dBeta;
    point tPoint;
    tPoint.z = 0.0;
    std::vector<double> xVector;
    std::vector<point> sPoints;
    std::vector<point> cPoints;
    std::vector<double> tDist;
    std::vector<double> gVector;
    xVector.reserve(n + 1);
    sPoints.reserve(n*2);
    cPoints.reserve(n + 1);
    tDist.reserve(n + 1);
    gVector.reserve(n + 1);
    dBeta = M_PI / n;
    for(size_t i = 0; i <= n; i++){
        xVector.push_back((1-std::cos(i*dBeta))*0.5);
    }
    if(foil.length() == 8){
        maxCamber = (float)(foil.at(4) - (int)'0') * 0.01;
        maxCamberPos = (float)(foil.at(5) - (int)'0') *0.1;
        thickness = (float)std::stoi(foil.substr(6, 2)) *0.01;
        for(size_t i = 0; i <= n; i++){
            tPoint.x = xVector[i];
            tDist.push_back((thickness/0.2)*(0.2969*std::sqrt(tPoint.x) - 0.126*tPoint.x - 0.3516*tPoint.x*tPoint.x + 0.2843*tPoint.x*tPoint.x*tPoint.x - 0.1036*tPoint.x*tPoint.x*tPoint.x*tPoint.x));
            if(tPoint.x < maxCamberPos){
                tPoint.y = (maxCamber/(maxCamberPos*maxCamberPos))*(2*maxCamberPos*tPoint.x - tPoint.x*tPoint.x);
                gVector.push_back(std::atan(((2*maxCamber)/(maxCamberPos*maxCamberPos))*(maxCamberPos - tPoint.x)));
            } else {
                tPoint.y = (maxCamber/((1 - maxCamberPos)*(1-maxCamberPos)))*(1.0  -2*maxCamberPos + 2*maxCamberPos*tPoint.x - tPoint.x*tPoint.x);
                gVector.push_back(((2*maxCamber)/((1-maxCamberPos)*(1-maxCamberPos)))*(maxCamberPos - tPoint.x));
            }
            cPoints.push_back(tPoint);
        }
        tDist[n] = 0.0;
        gVector[n] = 0.0;
        for(int i = n; i >= 0; i--){
            tPoint.x = cPoints[i].x - tDist[i]*std::sin(gVector[i]);
            tPoint.y = cPoints[i].y + tDist[i]*std::cos(gVector[i]);
            tPoint.idx = n-1-i;

            if(i == static_cast<int>(n)){
                tPoint.x = 1.0;
                tPoint.y = cPoints[n].y;
            }

            sPoints.push_back(tPoint);
        }
        for(size_t i = 1; i <= n; i++){
            tPoint.x = cPoints[i].x + tDist[i]*std::sin(gVector[i]);
            tPoint.y = cPoints[i].y - tDist[i]*std::cos(gVector[i]);
            tPoint.idx = n+i;

            if(i == n){
                tPoint.x = 1.0;
                tPoint.y = cPoints[n].y;
            }

            sPoints.push_back(tPoint);
        }

    } else {
        throw std::runtime_error("5 digit NACA not supported just yet");
        //float cl = (float)(foil.at(4) - (int)'0') * 0.15;
        //maxCamberPos = (float)(foil.at(5) - (int)'0') * 0.05;
        //bool isReflex = (bool)(foil.at(6) - (int)'0');
        //thickness = (float)std::stoi(foil.substr(7, 2)) *0.01;
    }
    this->points = sPoints;
}

inline bool pointIsBounded(const std::vector<airfoil>& data, const std::vector<double>& z, double zp, int idx){
    if(z.size() != data.size()){
        throw std::runtime_error("Coordinate and data vector sizes do not match");
    }
    size_t imin = std::distance(z.begin(), std::min_element(z.begin(), z.end()));
    size_t imax = std::distance(z.begin(), std::max_element(z.begin(), z.end()));
    double zmin = z[imin];
    double zmax = z[imax];
    size_t ib1 = 0;
    size_t ib2 = 0;
    double d;
    double d1 = zmax-zmin;
    double d2 = d1;
    if(zp <= zmin){
        return data[imin].pointIsInSection(idx);
    } else if (zp >= zmax) {
        return data[imax].pointIsInSection(idx);
    } else {
        for(size_t i = 0; i < z.size(); i++){
            d = std::abs(zp-z[i]);
            if(d < d1){
                ib2 = ib1;
                ib1 = i;
                d2 = d1;
                d1 = d;
            } else if (d < d2){
                ib2 = i;
                d2 = d;
            }
        }
        return data[ib1].pointIsInSection(idx) && data[ib2].pointIsInSection(idx);
    }
}

template<typename T>
SpanwiseVec<T>::SpanwiseVec(const std::vector<double>& coords, const std::vector<T>& SWData): z(coords), data(SWData){

    if(z.size() != data.size()){
        throw std::runtime_error("Size of coordinate and data vectors do not match");
    }
    if(z.size() < 2){
        throw std::runtime_error("Vector must have at least two elements");
    }
    this->isSet = true;

}

template<typename T>
void SpanwiseVec<T>::set(const std::vector<double>& zN, const std::vector<T>& dataN){
    
    if(zN.size() != dataN.size()){
        throw std::runtime_error("Size of coordinate and data vectors do not match");
    }
    if(zN.size() < 2){
        throw std::runtime_error("Vector must have at least two elements");
    }
    this->z = zN;
    this->data = dataN;
    this->isSet = true;
    this->coordsHaveBeenRead = false;
    this->dataHasBeenRead = false;

}

template<typename T>
void SpanwiseVec<T>::setNoInterp(const std::vector<double>& zN, const std::vector<T>& dataN){
    
    if(!this->isSet){
        throw std::runtime_error("Main coordinate and data vectors must be set before auxiliary vectors are defined.");
    }
    if(zN.size() != dataN.size()){
        throw std::runtime_error("Size of coordinate and data vectors do not match");
    }
    if(zN.size() < 2){
        throw std::runtime_error("Vector must have at least two elements");
    }
    this->z_noInterp = zN;
    this->data_noInterp = dataN;
    this->isInterpolated = true;

}

template<typename T>
void SpanwiseVec<T>::setZOrig(const std::vector<double>& zN){

    if(!this->isSet){
        throw std::runtime_error("Main coordinate and data vectors must be set before auxiliary vectors are defined.");
    }
    if(zN.size() < 2){
        throw std::runtime_error("Vector must have at least two elements");
    }
    this->z_orig = zN;
    this->isCoordNormalized = true;
    
}

template<typename T>
void SpanwiseVec<T>::updateCoordBounds(){

    if(!this->isSet){
        throw std::runtime_error("Full vector is not set yet");
    }
    this->z_max = *std::max_element(this->z.begin(), z.end());
    this->z_min = *std::min_element(this->z.begin(), z.end());
    this->CoordBoundsSet = true;

}

template<typename T>
void SpanwiseVec<T>::updateDataBounds(){

    if(!this->isSet){
        throw std::runtime_error("Full vector is not set yet");
    }
    if constexpr (std::is_same_v<T, double> || std::is_same_v<T, int> || std::is_same_v<T, float>){
        this->data_max = *std::max_element(this->data.begin(), data.end());
        this->data_min = *std::min_element(this->data.begin(), data.end());
    } else if constexpr (std::is_same_v<T, std::vector<double>> || std::is_same_v<T, std::vector<int>> || std::is_same_v<T, std::vector<float>>) {
        for(size_t i = 0; i < this->data.size(); i++){
            this->data_max[i] = *std::max_element(this->data[i].begin(), this->data[i].end());
            this->data_min[i] = *std::min_element(this->data[i].begin(), this->data[i].end());
        }
    } else if constexpr (std::is_same_v<T, airfoil>){
        this->data_max = this->data[this->data.size()-1];
        this->data_min = this->data[0];
    } else {
        throw std::runtime_error(std::string("Unsupported type for automatic bound detection: ") + typeid(T).name());
    }
    this->DataBoundsSet = true;
}

template<typename T>
void SpanwiseVec<T>::setCoordBounds(double a, double b){
    if(a >= b){
        throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
    }
    this->z_min = a;
    this->z_max = b;
    this->CoordBoundsSet = true;
}

template<typename T>
void SpanwiseVec<T>::setScalarDataBounds(double a, double b){
    if( a >= b){
        throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
    }
    for(const auto& val : this->data_min){
        val = a; 
    }
    for(const auto& val : this->data_max){
        val = b;
    }
    this->DataBoundsSet = true;
}

template<typename T>
void SpanwiseVec<T>::setDataBounds(T a, T b){
    
    if constexpr (std::is_same_v<T, double> || std::is_same_v<T, int> || std::is_same_v<T, float>){
        if(a >= b){
            throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
        }
    } else if constexpr (std::is_same_v<T, std::vector<double>> || std::is_same_v<T, std::vector<int>> || std::is_same_v<T, std::vector<float>>) {
        for(size_t i = 0; i < this->data.size(); i++){
            if(a[i] >= b[i]){
                throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
            }
        }
    }
    this->data_min = a;
    this->data_max = b;
    this->DataBoundsSet = true;

}

template<typename T>
void SpanwiseVec<T>::setOrigCoordBounds(double a, double b){
    if(a >= b){
        throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
    }
    this->z_min_orig = a;
    this->z_max_orig = b;
}

template<typename T>
void SpanwiseVec<T>::setOrigScalarDataBounds(double a, double b){
    if( a >= b){
        throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
    }
    for(const auto& val : this->data_min_orig){
        val = a; 
    }
    for(const auto& val : this->data_max_orig){
        val = b;
    }
}

template<typename T>
void SpanwiseVec<T>::setOrigDataBounds(const T a, T b){
    
    if constexpr (std::is_same_v<T, double> || std::is_same_v<T, int> || std::is_same_v<T, float>){
        if(a >= b){
            throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
        }
    } else if constexpr (std::is_same_v<T, std::vector<double>> || std::is_same_v<T, std::vector<int>> || std::is_same_v<T, std::vector<float>>) {
        for(size_t i = 0; i < this->data.size(); i++){
            if(a[i] >= b[i]){
                throw std::runtime_error("Invalid range. Lower bound must be less than upper bound");
            }
        }
    }
    this->data_min_orig = a;
    this->data_max_orig = b;

}


template<typename T>
void SpanwiseVec<T>::normalizeCoords(){

    if(!this->CoordBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Update or set manually");
    }
    this->z_orig = this->z;
    this->z_min_orig = this->z_min;
    this->z_max_orig = this->z_max;
    this->z = uasisi::normalize(this->z, this->z_min , this->z_max, -1.0, 1.0);
    this->isCoordNormalized = true;
    this->setCoordBounds(-1.0, 1.0);

}

template<typename T>
void SpanwiseVec<T>::normalizeCoordsFrom(double a, double b){

    if(!this->CoordBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->z_orig = this->z;
    this->z_min_orig = a;
    this->z_max_orig = b;
    this->z = uasisi::normalize(this->z, a, b, -1.0, 1.0);
    this->isCoordNormalized = true;
    this->setCoordBounds(-1.0, 1.0);

}

template<typename T>
void SpanwiseVec<T>::normalizeCoordsTo(double a, double b){

    if(!this->CoordBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->z_orig = this->z;
    this->z_min_orig = this->z_min;
    this->z_max_orig = this->z_max;
    this->z = uasisi::normalize(this->z, this->z_min , this->z_max, a, b);
    this->isCoordNormalized = true;
    this->setCoordBounds(a, b);

}

template<typename T>
void SpanwiseVec<T>::normalizeCoordsFromTo(double a, double b, double c, double d){

    if(!this->CoordBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->z_orig = this->z;
    this->z_min_orig = a;
    this->z_max_orig = b;
    this->z = uasisi::normalize(this->z, a, b, c, d);
    this->isCoordNormalized = true;
    this->setCoordBounds(c, d);

}

template<typename T>
void SpanwiseVec<T>::normalizeData(){

    if(!this->DataBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->data_min_orig = data_min;
    this->data_max_orig = data_max;
    this->data = uasisi::normalize(this->data, data_min, data_max, -1.0, 1.0);
    this->isDataNormalized = true;
    this->setScalarDataBounds(-1.0, 1.0);

}

template<typename T>
void SpanwiseVec<T>::normalizeDataFrom(T a, T b){

    if(!this->DataBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->data_min_orig = a;
    this->data_max_orig = b;
    this->data = uasisi::normalize(this->data, a, b, -1.0, 1.0);
    this->isDataNormalized = true;
    this->setScalarDataBounds(-1.0, 1.0);

}

template<typename T>
void SpanwiseVec<T>::normalizeDataTo(T a, T b){

    if(!this->DataBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->data_min_orig = data_min;
    this->data_max_orig = data_max;
    this->data = uasisi::normalize(this->data, data_min, data_max, a, b);
    this->isDataNormalized = true;
    this->setDataBounds(a, b);

}

template<typename T>
void SpanwiseVec<T>::normalizeDataFromTo(T a, T b, T c, T d){

    if(!this->DataBoundsSet){
        throw std::runtime_error("Cannot normalize vector with unset bounds. Use update() or set()");
    }
    this->data_min_orig = a;
    this->data_max_orig = b;
    this->data = uasisi::normalize(this->data, a, b, c, d);
    this->isDataNormalized = true;
    this->setDataBounds(c, d);

}

template<typename T>
void SpanwiseVec<T>::denormalizeCoords(){

    if(this->isCoordNormalized){
        this->normalizeCoordsFromTo(this->z_min, this->z_max, this->z_min_orig, this->z_max_orig);
        this->isCoordNormalized = false;
    } else {
        std::cout << "Coordinates not normalized, ignoring\n";
    }

}

template<typename T>
void SpanwiseVec<T>::denormalizeData(){

    if(this->isDataNormalized){
        this->normalizeDataFromTo(this->data_min, this->data_max, this->data_min_orig, this->data_max_orig);
        this->isDataNormalized = false;
    } else {
        std::cout << "Data not normalized, ignoring\n";
    }

}

template<typename T>
void SpanwiseVec<T>::interpolateData(std::vector<double> zp, interpType t){
    if(!this->isSet){
        throw std::runtime_error("Cannot interpolate unset vector");
    }
    this->z_noInterp = this->z;
    this->data_noInterp = this->data;
    this->z = zp;
    this->data = interpolate(this->z_noInterp, this->data_noInterp, this->z, getInterpType(t));
    this->isInterpolated = true;
}

template<typename T>
void SpanwiseVec<T>::uninterpolate(){

    if(this->isInterpolated){
        this->z = this->z_noInterp;
        this->data = this->data_noInterp;
        this->isInterpolated = false;
    } else {
        std::cout << "Spanwise vector not interpolated, ignoring\n";
    }
}

template<typename T>
Eigen::Map<const Eigen::VectorXd> SpanwiseVec<T>::SWDataAsEigen() const{
    if constexpr (std::is_same_v<T, double>) {
        return vecAsEigen(this->SWData()); 
    } else {
        throw std::runtime_error("This method is only for scalar valued vectors. Please use the correct method");
    }
}

template<typename T>
Eigen::Map<Eigen::VectorXd> SpanwiseVec<T>::SWDataAsEigenMutable(){
    if constexpr (std::is_same_v<T, double>) {
        return vecAsEigenMutable(this->SWData()); 
    } else {
        throw std::runtime_error("This method is only for scalar valued vectors. Please use the correct method");
    }
}
template<typename T>
template<typename U, typename>
Eigen::VectorXd SpanwiseVec<T>::SWDataMemberAsEigen(double (U::*getter)() const) const{
    std::vector<double> member = extractMemberVector(this->SWData(), getter);
    return vecAsEigen(member);
}

template<typename T>
Eigen::VectorXd SpanwiseVec<T>::SWDataRowAsEigen(size_t idx) const{
    if constexpr (std::is_same_v<T, std::vector<double>>) {
        std::vector<double> row = extractRowVector(this->SWData(), idx);
        return vecAsEigen(row);
    } else {
        throw std::runtime_error("This method is only for vectors. Please use the correct method");
    }
}

inline const gsl_interp_type* getInterpType(interpType t){
    switch(t){
        case interpType::LIN:
            return gsl_interp_linear;
        case interpType::AKI:
            return gsl_interp_akima;
        case interpType::CSP:
            return gsl_interp_cspline;
        default:
            throw std::runtime_error("Unknown interpolation type.");
    }
}

template<typename T, typename U, typename W>
std::vector<T> normalize(const std::vector<T>& vec, const U& a, const U& b, const W& c, const W& d){
    if constexpr (std::is_same_v<T, double> && std::is_same_v<U, double> && std::is_same_v<W, double>){

        if(a >= b || c >= d){
            throw std::runtime_error("Error, invalid range");
        }
        double scale = (d-c) / (b-a);
        double offset = c-a*scale;
        std::vector<T> nVec;
        nVec.reserve(vec.size());
        for(const auto& val : vec){
            nVec.push_back(val*scale + offset);
        }
        return nVec;

    } else if constexpr (std::is_same_v<T, std::vector<double>> && std::is_same_v<U, double> && std::is_same_v<W, double>){

        std::vector<T> nVec;
        nVec.reserve(vec.size());
        for(const auto& val : vec){
           nVec.push_back(normalize(val, a, b, c, d));
        }
        return nVec;

    } else if constexpr (std::is_same_v<T, std::vector<double>> && std::is_same_v<U, std::vector<double>> && std::is_same_v<W, double>){

        std::vector<T> nVec;
        nVec.reserve(vec.size());
        for(size_t i = 0; i < vec.size(); i++){
           nVec.push_back(normalize(vec[i], a[i], b[i], c, d)); 
        }
        return nVec;

    } else if constexpr (std::is_same_v<T, std::vector<double>> && std::is_same_v<U, double> && std::is_same_v<W, std::vector<double>>){
        
        std::vector<T> nVec;
        nVec.reserve(vec.size());
        for(size_t i = 0; i < vec.size(); i++){
           nVec.push_back(normalize(vec[i], a, b, c[i], d[i])); 
        }
        return nVec;
    
    } else if constexpr (std::is_same_v<T, std::vector<double>> && std::is_same_v<U, std::vector<double>> && std::is_same_v<W, std::vector<double>>){
        
        std::vector<T> nVec;
        nVec.reserve(vec.size());
        for(size_t i = 0; i < vec.size(); i++){
           nVec.push_back(normalize(vec[i], a[i], b[i], c[i], d[i])); 
        }
        return nVec;

    } else {
    
        throw std::runtime_error(std::string("Unsupported types for normalization: ") + typeid(T).name() +  ", " + typeid(U).name() + "and " + typeid(W).name());

    }
}

template<typename T>
std::vector<T> interpolate(const std::vector<double>& z, const std::vector<T>& data, const std::vector<double>& zp, const gsl_interp_type* t){
    size_t min_points = (t == gsl_interp_akima) ? 5 : 2;
    if (z.size() < min_points){
        throw std::runtime_error("Not enough data points for interpolation with selected method.");
    }
    if constexpr (std::is_same_v<T, double>){
        std::vector<T> nData;
        nData.reserve(zp.size());

        gsl_interp_accel *acc = gsl_interp_accel_alloc();
        gsl_spline *spline = gsl_spline_alloc(t, z.size());
        gsl_spline_init(spline, z.data(), data.data(), z.size());

        for(double val : zp){
            nData.push_back(gsl_spline_eval(spline, val, acc));
        }

        gsl_spline_free(spline);
        gsl_interp_accel_free(acc);
        
        return nData;
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {

        std::vector<T> nData;
        nData.reserve(data.size());

        for(const T& vec : data){
            nData.push_back(interpolate(z, vec, zp, t));
        }

        return nData;

    } else if constexpr (std::is_same_v<T, airfoil>) {

        std::vector<T> nData;
        nData.resize(zp.size()); //interpolate chords, att points, etc in place.

        std::vector<double> chords;
        chords.reserve(zp.size());
        std::vector<point> attPoints;
        attPoints.reserve(zp.size());

        chords = interpolate(z, extractMemberVector(data, &airfoil::getChord), zp, t);
        attPoints = interpolate(z, extractMemberVector(data, &airfoil::getAttatchmentPoint), zp, t);

        std::set<int> pointIDset;

        point tPoint;
        std::vector<point> tPoints;
        std::vector<double> tZp;

        tPoints.reserve(1);
        tZp.reserve(1);

        for(size_t i = 0; i < z.size(); i++){
            for(const point& p : data[i].coords()){
                pointIDset.insert(p.idx);
            }
        }

        std::vector<int> pointIDs(pointIDset.begin(), pointIDset.end());

        std::vector<double> tZ;
        std::vector<point> tData;

        tZ.reserve(z.size());
        tData.reserve(z.size());

        for(int val : pointIDs){
            for(size_t i = 0; i < z.size(); i++){
                if(data[i].pointIsInSection(val)){
                    tZ.push_back(z[i]);
                    tData.push_back(data[i].getPoint(val));
                }
            }

            for(size_t i = 0; i < zp.size(); i++){
                if(pointIsBounded(data, z, zp[i], val)){
                    tZp.push_back(zp[i]);
                    tPoints = interpolate(tZ, tData, tZp, t);
                    tPoint = tPoints[0];
                    tPoint.idx = val;
                    nData[i].pushPoint(tPoint); 
                }
                tZp.clear();
            }
            tZ.clear();
            tData.clear();

        }

        for(size_t i = 0; i < zp.size(); i++){
            nData[i].setChord(chords[i]);
            nData[i].setAttatchmentPoint(attPoints[i]);
        }

        return nData;


    } else if constexpr (std::is_same_v<T, point>) {

        std::vector<T> nData;
        nData.reserve(zp.size());
        
        std::vector<double> xData = extractVariableVector(data, &point::x);
        std::vector<double> yData = extractVariableVector(data, &point::y);
        std::vector<double> zData = extractVariableVector(data, &point::z);

        gsl_interp_accel *accX = gsl_interp_accel_alloc();
        gsl_spline *splineX = gsl_spline_alloc(t, z.size());
        gsl_interp_accel *accY = gsl_interp_accel_alloc();
        gsl_spline *splineY = gsl_spline_alloc(t, z.size());
        gsl_interp_accel *accZ = gsl_interp_accel_alloc();
        gsl_spline *splineZ = gsl_spline_alloc(t, z.size());
        gsl_spline_init(splineX, z.data(), xData.data(), z.size());
        gsl_spline_init(splineY, z.data(), yData.data(), z.size());
        gsl_spline_init(splineZ, z.data(), zData.data(), z.size());

        point tPoint;

        for(double val : zp){
            tPoint.x = gsl_spline_eval(splineX, val, accX);
            tPoint.y = gsl_spline_eval(splineY, val, accY);
            tPoint.z = gsl_spline_eval(splineZ, val, accZ);
            nData.push_back(tPoint);
        }
        
        gsl_spline_free(splineX);
        gsl_interp_accel_free(accX);
        gsl_spline_free(splineY);
        gsl_interp_accel_free(accY);
        gsl_spline_free(splineZ);
        gsl_interp_accel_free(accZ);

        return nData;

    } else {
        throw std::runtime_error("Cannot interpolate this data type");
    }
}

template<typename T, typename U>
std::vector<T> extractMemberVector(const std::vector<U>& objects, T (U::*getter)() const){

    std::vector<T> result;
    result.reserve(objects.size());
    std::transform(objects.begin(), objects.end(), std::back_inserter(result), [getter](const U& obj) {return (obj.*getter)(); });
    return result;

}

template<typename T, typename U>
std::vector<T> extractVariableVector(const std::vector<U>& objects, T U::*member){

    std::vector<T> result;
    result.reserve(objects.size());
    std::transform(objects.begin(), objects.end(), std::back_inserter(result), [member](const U& obj) {return obj.*member;});
    return result;

}

template<typename T>
std::vector<T> extractRowVector(const std::vector<std::vector<T>>& objects, size_t idx){
    
    std::vector<T> result;
    result.reserve(objects.size());
    if(objects.empty()){
        throw std::runtime_error("Empty mother vector");
    }
    double size = objects[0].size();
    for(size_t i = 1; i < objects.size(); i++){
        if(objects[i].size() != size){
            throw std::runtime_error("Size mismatch");
        }
    }
    if(idx >= size){
        throw std::runtime_error("Index out of range");
    }
    for(const std::vector<T>& vec : objects){
        result.push_back(vec[idx]);
    }
    return result;

}

inline Eigen::Map<const Eigen::VectorXd> vecAsEigen(const std::vector<double>& x){
    return Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
}

inline Eigen::Map<Eigen::VectorXd> vecAsEigenMutable(std::vector<double>& x){
    return Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
}

inline std::vector<double> EigenAsVec(const Eigen::VectorXd& x){
    std::vector<double> vec;
    vec.assign(x.data(), x.data() + x.size());
    return vec;
}

inline void updateVecWithEigen(const Eigen::VectorXd& x, std::vector<double>& vec){// non const reference is intended
    vec.assign(x.data(), x.data() + x.size()); //I think this is correct but maybe I have to clear the vector first?
}

}

#endif
