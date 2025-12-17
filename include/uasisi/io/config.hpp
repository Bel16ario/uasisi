#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <vector>

namespace uasisi{

enum class MWType {
    AOA,
    CAMBER,
    BOTH,
    NONE
};

class Config {
public:

    Config(const std::string& filename);
    ~Config();

    double                  getWingSpan() const {return wingSpan;}
    int                     getNActuators() const {return nActuators;}
    double                  getOmega() const {return Omega;}
    double                  getDelta() const {return Delta;}
    MWType                  getMWType() const {return mWType;}
    bool                    getActLinSpaced() const {return actLinSpaced;}
    std::vector<double>     getActCoords() const {return actCoords;}
    double                  getVInf() const {return vInf;}
    double                  getRho0() const {return rho0;}
    double                  getMu0() const {return mu0;}
    double                  getAlpha() const {return alpha;}
    double                  getDt() const {return dt;}
    double                  getTMax() const {return tMax;}

private:
    //Wing characteristics
    double                  wingSpan;       //Length of Wing
    void                    setWingSpan(){this->wingSpan = read<double>("wingSpan", root);}
    int                     nActuators;     //Number of actuators on Wing 
    void                    setNActuators(){this->nActuators = read<int>("nActuators", root);}
    double                  Omega;          //Máximum actuator torsion
    void                    setOmega(){this->Omega = read<double>("Omega", root);}
    double                  Delta;          //Maximum actuator camber
    void                    setDelta(){this->Delta = read<double>("Delta", root);}
    MWType                  mWType;         //Morphic wing type
    void                    setMWType(){
        std::string wType = read<std::string>("mWType", root);
        if(wType == "AOA") this->mWType = MWType::AOA;
        else if(wType == "CAMBER") this->mWType = MWType::CAMBER;
        else if(wType == "BOTH") this->mWType = MWType::BOTH;
        else if(wType == "NONE") this->mWType = MWType::NONE;
        else throw std::runtime_error("Incorrect morphic wing type (mWType): " + wType + "\n AOA, CAMBER, BOTH, NONE");
    }
    //Geometry descriptors;
    bool                    actLinSpaced;
    void                    setActLinSpaced(){this->actLinSpaced = read<bool>("actLinSpaced", root);}
    std::vector<double>     actCoords;
    void                    setActCoords(){this->actCoords = read<std::vector<double>>("actCoords", root);}
    //Flight conditions
    double                  vInf;           //Flow velocity
    void                    setVInf(){this->vInf = read<double>("vInf", root);}
    double                  rho0;           //Fluid density
    void                    setRho0(){this->rho0 = read<double>("rho0", root);}
    double                  mu0;            //Dynamic viscocity
    void                    setMu0(){this->mu0 = read<double>("mu0", root);}
    double                  alpha;          //Angle of attack
    void                    setAlpha(){this->alpha = read<double>("alpha", root);}
    //Simulation settings
    double                  dt;             //Time increment
    void                    setDt(){this->dt = read<double>("dt", root);}
    double                  tMax;           //Maximum simulation time
    void                    setTMax(){this->tMax = read<double>("tMax", root);}
    //Config parser
    YAML::Node root;
    template <typename T>
    T read(const std::string& key, YAML::Node node) const{
        if(node[key]){
            return node[key].as<T>();
        } else {
            throw std::runtime_error("Key not found: " + key);
        }
    }
};

}

#endif
