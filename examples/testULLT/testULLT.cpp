// Prueba aislada de ULLTPhy (sin orquestador para el ULLT; el estacionario de PhillipsPhy si lo usa).
// Perfil: c_alpha = 6.5317, alpha_0L = -0.0375 -> c0 = -c_alpha alpha_0L (mismos valores que PhillipsPhy).
// Caso 1: envergadura muy grande -> w_y ~ 0 -> l/(q c (c_alpha alpha + c0)) debe seguir a Wagner.
// Caso 2: AR = 10 -> el estado estacionario debe coincidir con Multhopp estacionario.
// Caso 3: pulso gaussiano de cabeceo en una estacion, partiendo del estacionario de PhillipsPhy.
// Caso 4: cabeceo senoidal en la misma estacion, partiendo del mismo estacionario.
// Las senales del ULLT se conectan a mano, igual que lo haria el orquestador, y SimpleLogger
// las graba en ../results/ullt_*.h5.

#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "uasisi/core/types.hpp"
#include "uasisi/core/orchestrator.hpp"
#include "physics/uLLTPhy/uLLTPhy.hpp"   // TODO: ajustar ruta si el modulo vive en otra carpeta
#include "physics/phillipsPhy/phillipsPhy.hpp"
#include "control/constantCtrl/constantCtrl.hpp"
#include "actuators/constantAccelAct/constantAccelAct.hpp"
#include "monitors/simpleLogger/simpleLogger.hpp"

using namespace uasisi;

constexpr double CHORD = 1.0;
constexpr double CALPHA = 6.5317;
constexpr double ALPHA0L = -0.0375;
constexpr double C0 = -CALPHA*ALPHA0L;   // c_l = c_alpha (alpha - alpha_0L) = c_alpha alpha + c0
constexpr double DR = 0.5;               // eje en c/4: d_r = +0.5 semicuerdas (positivo hacia el BA)
constexpr double ALTITUDE = 1000.0;      // m, como en los ejemplos
double RHO = 1.225;                      // se toma de FlightConditions en main()

double toRad(double deg){ return deg*M_PI/180.0; }

airfoil makeAirfoil(){
    airfoil af;
    af.setChord(CHORD);
    af.setCAlpha(CALPHA);
    af.setC0(C0);
    af.setAttatchmentPoint(point{DR, 0.0, 0.0, 0});
    return af;
}

std::vector<double> betaCoords(size_t n){
    std::vector<double> beta(n);
    for(size_t i = 0; i < n; i++) beta[i] = i*M_PI/(n - 1);
    return beta;
}

std::vector<double> yCoords(size_t n, double span){
    std::vector<double> y(n);
    for(size_t i = 0; i < n; i++) y[i] = -0.5*span*std::cos(i*M_PI/(n - 1));
    return y;
}

ULLTConfig makeConfig(size_t n, double span){
    ULLTConfig cfg;
    cfg.nPts = n;
    cfg.span = span;
    cfg.rho = RHO;
    cfg.cRef = CHORD;
    cfg.du = 0.0;
    cfg.integrator = IntegratorScheme::Euler;
    // Wagner (Jones): Phi = 1 - sum a e^{lam s}, sum a = 0.5
    cfg.aWag   = Eigen::RowVector2d(0.165, 0.335).replicate(n, 1);
    cfg.lamWag = Eigen::RowVector2d(-0.0455, -0.3).replicate(n, 1);
    // Gennaretti, ajuste de 3 terminos a gennaretti.dat (sum a = 1, error max 0.018)
    cfg.aGen   = Eigen::RowVector3d(0.2039, 0.5306, 0.2655).replicate(n, 1);
    cfg.lamGen = Eigen::RowVector3d(-0.3711, -1.3562, -15.3536).replicate(n, 1);
    cfg.chiInit = Eigen::MatrixXd::Zero(n, 2);
    cfg.xiInit  = Eigen::MatrixXd::Zero(n, 3);
    cfg.psiInit = Eigen::MatrixX3d::Zero(n, 3);
    return cfg;
}

// Estados de retraso en equilibrio para una distribucion de sustentacion estacionaria l [N/m]:
//   chi_j = -sigma/lam_j, sigma = c_alpha alpha_e + c0 = l/(q c);   xi_k = -l/lamHat_k
void setSteadyStates(ULLTConfig& cfg, const Eigen::VectorXd& l, double q){
    const Eigen::Index n = l.size();
    for(Eigen::Index i = 1; i < n - 1; i++){
        const double sigma = l(i)/(q*CHORD);
        cfg.chiInit.row(i) = -sigma*cfg.lamWag.row(i).cwiseInverse();
        cfg.xiInit.row(i)  = -l(i)*cfg.lamGen.row(i).cwiseInverse();
    }
}

// Multhopp estacionario del propio modelo: (I + k W) g = k (alpha + c0/c_alpha), k = c c_alpha/(2b)
Eigen::VectorXd steadyLift(size_t n, double b, double u, double alpha){
    Eigen::VectorXd z(n);
    for(size_t i = 0; i < n; i++) z(i) = i*M_PI/(n - 1);
    const size_t m = n - 2;
    const double M1 = n - 1.0;
    Eigen::MatrixXd W(m, m);
    for(size_t i = 1; i < n - 1; i++){
        for(size_t j = 1; j < n - 1; j++){
            const size_t d = (j > i) ? j - i : i - j;
            if(i == j)          W(i-1, j-1) = M1/(4*std::sin(z(i)));
            else if(d % 2 == 0) W(i-1, j-1) = 0.0;
            else                W(i-1, j-1) = -std::sin(z(j))/std::pow(std::cos(z(j)) - std::cos(z(i)), 2)/M1;
        }
    }
    const double k = CHORD*CALPHA/(2*b);
    const Eigen::VectorXd g = (Eigen::MatrixXd::Identity(m, m) + k*W)
                                  .partialPivLu().solve(Eigen::VectorXd::Constant(m, k*(alpha + C0/CALPHA)));
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n);
    l.segment(1, m) = RHO*b*u*u*g;       // l = rho u gamma, gamma = g b u
    return l;
}

// Estacionario de PhillipsPhy en las estaciones del ULLT (torsion nula, alpha de vInf = 0).
// Misma cadena que openLoopLLTControl: constantCtrl -> constantAccelAct -> phillipsPhy.
// El actuador arranca en su objetivo (0), asi que init() ya da el estacionario.
Eigen::VectorXd phillipsSteadyLift(size_t n, double span, double u){
    std::vector<double> zPts = yCoords(n, span);
    std::vector<double> zActs = {-0.5*span, -0.25*span, 0.0, 0.25*span, 0.5*span};
    const size_t nA = zActs.size();

    FlightConditions fConds;
    fConds.setVInf(point{u, 0.0, 0.0, 0});
    fConds.setAltitude(ALTITUDE);

    Orchestrator orch;
    orch.createModule("constantCtrl", "ctrl", 0, [&](IModule* mod){
        auto* p = dynamic_cast<ConstantCtrl*>(mod);
        if(!p) throw std::runtime_error("Problem casting from IModule class");
        p->setIType(interpType::LIN);
        p->setDType(DataType::DOB);
        p->setZ(std::vector<double>{-0.5*span, 0.5*span});
        p->setSData(std::vector<double>{0.0, 0.0});
        p->setActCoords(zActs);
    });
    orch.createModule("constantAccelAct", "act", 1, [&](IModule* mod){
        auto* p = dynamic_cast<ConstantAccelAct*>(mod);
        if(!p) throw std::runtime_error("Problem casting from IModule class");
        p->setIType(interpType::LIN);
        p->setZ(zActs);
        p->setZOut(zPts);
        p->setMaxPos(std::vector<double>(nA, toRad(5.0)));
        p->setMaxVel(std::vector<double>(nA, 0.5));
        p->setMaxAccel(std::vector<double>(nA, 0.2));
        p->setCenterPos(std::vector<double>(nA, 0.0));
        p->setTheta(std::vector<double>(nA, 0.0));
        p->setOmega(std::vector<double>(nA, 0.0));
        p->setThickness(0.05);
        p->setAddThickness(true);
    });
    orch.createModule("phillipsPhy", "phy", 2, [&](IModule* mod){
        auto* p = dynamic_cast<PhillipsPhy*>(mod);
        if(!p) throw std::runtime_error("Problem casting from IModule class");
        p->setIType(interpType::LIN);
        p->setZOut(zPts);
        p->setNSWPoints(200);
        p->updateFConds(fConds);
    });
    orch.configureModules();
    orch.connectSystem();
    orch.init();

    const SpanwiseVec<double>* lp = orch.getDOB("realLift");
    if(lp->size() != n) throw std::runtime_error("PhillipsPhy devolvio un numero de puntos distinto");
    Eigen::VectorXd l(n);
    for(size_t i = 0; i < n; i++) l(i) = (*lp)[i];
    l(0) = 0.0;
    l(n - 1) = 0.0;
    return l;
}

// ---------------------------------------------------------------------------------------------
// Casos 1 y 2: escalon de rootAoA en s = 0, partiendo de cero
// ---------------------------------------------------------------------------------------------
struct Result{
    std::vector<double> s, lMid, L;
};

Result runStep(double span, size_t n, double u, double alpha, double sMax, double ds, const std::string& fileName){
    const std::vector<double> beta = betaCoords(n);
    SpanwiseVec<airfoil> geom;
    geom.set(beta, std::vector<airfoil>(n, makeAirfoil()));

    double airspeed = u;
    double rootAoA = alpha;
    SpanwiseVec<double> lift;
    double L = 0.0, Mx = 0.0;

    ULLTPhy phy;
    phy.configure(makeConfig(n, span));
    phy.connectInputAirfoil("realGeometry", &geom);
    phy.connectInputScalar("airspeed", &airspeed);
    phy.connectInputScalar("rootAoA", &rootAoA);
    phy.connectOutputDouble("realLift", &lift);
    phy.connectOutputScalar("realTotalLift", &L);
    phy.connectOutputScalar("realRollingMoment", &Mx);

    // Logger: mismas llamadas que hace Orchestrator::connectSignal para un modulo de entrada.
    // Se declara despues de las senales para destruirse antes que ellas.
    SimpleLogger mon;
    mon.setFileName(fileName);
    mon.setWriteInterval(100);
    mon.connectInputDouble("realLift", &lift);
    mon.connectInputScalar("realTotalLift", &L);
    mon.connectInputScalar("realRollingMoment", &Mx);
    mon.connectInputScalar("airspeed", &airspeed);
    mon.connectInputScalar("rootAoA", &rootAoA);

    // Orden del orquestador: validar todo, luego init en orden de prioridad (fisica antes que monitor)
    phy.validateConnections();
    mon.validateConnections();
    phy.init();
    mon.init();

    const double dt = ds*CHORD/(2*u);     // ds = 2 u dt / c
    const size_t nSteps = static_cast<size_t>(std::lround(sMax/ds));
    const size_t mid = n/2;

    Result r;
    for(size_t k = 0; k <= nSteps; k++){
        r.s.push_back(k*ds);
        r.lMid.push_back(lift[mid]);
        r.L.push_back(L);
        if(k < nSteps){
            const double t = (k + 1)*dt;  // como en los ejemplos: t al final del paso
            phy.step(t, dt);
            mon.step(t, dt);
        }
    }
    return r;
}

// ---------------------------------------------------------------------------------------------
// Casos 3 y 4: cabeceo prescrito alpha(s) en la estacion mas cercana a y0, desde un estacionario
//   El modulo recibe la aceleracion fisica: d2alpha/dt2 = (2u/c)^2 d2alpha/ds2 (u constante).
//   Se envia la aceleracion MEDIA del paso, (alpha'(s) - alpha'(s - ds))/ds, no la instantanea:
//   el modulo integra dos veces la aceleracion, y con Euler el valor instantaneo al final del paso
//   deja un sesgo constante en psi_2 que hace derivar la torsion psi_1 (~4 % de la amplitud en el caso 4).
//   realTwist (la torsion prescrita) se graba junto con la sustentacion para visualizarlo.
//   wy = 0 -> solo una estacion; wy > 0 -> forma gaussiana en y de ancho wy [m].
// ---------------------------------------------------------------------------------------------
struct PitchForcing{
    std::function<double(double)> alpha;     // alpha(s)
    std::function<double(double)> alphaS;    // d alpha/ds
};

struct Series{
    size_t i0;
    double y0;
    std::vector<double> s, alpha, l, lNeighbour, L;
};

Series runPitchDemo(double span, size_t n, double u, double y0Target, double wy, const PitchForcing& f,
                    const Eigen::VectorXd& lInit, double sMax, double ds, int writeInterval,
                    const std::string& fileName){
    const std::vector<double> beta = betaCoords(n);
    const std::vector<double> y = yCoords(n, span);
    const double q = 0.5*RHO*u*u;

    size_t i0 = 1;
    for(size_t i = 1; i < n - 1; i++){
        if(std::abs(y[i] - y0Target) < std::abs(y[i0] - y0Target)) i0 = i;
    }
    std::vector<double> shape(n, 0.0);
    for(size_t i = 1; i < n - 1; i++){
        shape[i] = (wy > 0.0) ? std::exp(-std::pow((y[i] - y0Target)/wy, 2)) : (i == i0 ? 1.0 : 0.0);
    }

    ULLTConfig cfg = makeConfig(n, span);
    setSteadyStates(cfg, lInit, q);
    for(size_t i = 0; i < n; i++){
        cfg.psiInit(i, 0) = f.alpha(0.0)*shape[i];
        cfg.psiInit(i, 1) = f.alphaS(0.0)*shape[i];
    }

    SpanwiseVec<airfoil> geom;
    geom.set(beta, std::vector<airfoil>(n, makeAirfoil()));

    const double sToT2 = std::pow(2*u/CHORD, 2);      // d2/dt2 = (2u/c)^2 d2/ds2
    std::vector<double> twist(n), accel(n);
    auto fill = [&](double s, double sPrev){
        const double aMean = (f.alphaS(s) - f.alphaS(sPrev))/(s - sPrev);
        for(size_t i = 0; i < n; i++){
            twist[i] = f.alpha(s)*shape[i];
            accel[i] = sToT2*aMean*shape[i];
        }
    };
    fill(ds, 0.0);              // solo afecta al l_nc del cuadro inicial
    for(size_t i = 0; i < n; i++) twist[i] = f.alpha(0.0)*shape[i];
    SpanwiseVec<double> realTwist, realPitchAccel;
    realTwist.set(beta, twist);
    realPitchAccel.set(beta, accel);

    double airspeed = u;
    double rootAoA = 0.0;
    SpanwiseVec<double> lift;
    double L = 0.0, Mx = 0.0;

    ULLTPhy phy;
    phy.configure(cfg);
    phy.connectInputAirfoil("realGeometry", &geom);
    phy.connectInputScalar("airspeed", &airspeed);
    phy.connectInputScalar("rootAoA", &rootAoA);
    phy.connectInputDouble("realPitchAccel", &realPitchAccel);
    phy.connectOutputDouble("realLift", &lift);
    phy.connectOutputScalar("realTotalLift", &L);
    phy.connectOutputScalar("realRollingMoment", &Mx);

    SimpleLogger mon;
    mon.setFileName(fileName);
    mon.setWriteInterval(writeInterval);
    mon.connectInputDouble("realLift", &lift);
    mon.connectInputDouble("realTwist", &realTwist);
    mon.connectInputDouble("realPitchAccel", &realPitchAccel);
    mon.connectInputScalar("realTotalLift", &L);
    mon.connectInputScalar("realRollingMoment", &Mx);

    phy.validateConnections();
    mon.validateConnections();
    phy.init();
    mon.init();

    const double dt = ds*CHORD/(2*u);
    const size_t nSteps = static_cast<size_t>(std::lround(sMax/ds));

    Series r;
    r.i0 = i0;
    r.y0 = y[i0];
    auto record = [&](double s){
        r.s.push_back(s);
        r.alpha.push_back(twist[i0]);
        r.l.push_back(lift[i0]);
        r.lNeighbour.push_back(lift[i0 + 1]);
        r.L.push_back(L);
    };
    record(0.0);
    for(size_t k = 1; k <= nSteps; k++){
        const double s = k*ds;
        fill(s, s - ds);
        realTwist.set(beta, twist);
        realPitchAccel.set(beta, accel);
        phy.step(k*dt, dt);
        mon.step(k*dt, dt);
        record(s);
    }
    return r;
}

// Amplitud y fase de la componente a frecuencia k de x(s) sobre [s1, s2] (periodos enteros)
void harmonic(const std::vector<double>& s, const std::vector<double>& x, double k, double s1, double s2,
              double& amp, double& phaseDeg, double& mean){
    double a = 0, b = 0, m = 0;
    size_t cnt = 0;
    for(size_t i = 0; i < s.size(); i++){
        if(s[i] < s1 || s[i] >= s2) continue;
        m += x[i];
        cnt++;
    }
    mean = m/cnt;
    for(size_t i = 0; i < s.size(); i++){
        if(s[i] < s1 || s[i] >= s2) continue;
        a += (x[i] - mean)*std::sin(k*s[i]);
        b += (x[i] - mean)*std::cos(k*s[i]);
    }
    a *= 2.0/cnt;
    b *= 2.0/cnt;
    amp = std::hypot(a, b);
    phaseDeg = std::atan2(b, a)*180.0/M_PI;   // x ~ amp sin(k s + fase)
}

int main(){
    std::filesystem::create_directories("../results");

    // Silenciar los mensajes de los modulos (std::cout); los resultados usan printf
    auto* coutOld = std::cout.rdbuf();
    std::ofstream devNull("/dev/null");
    std::cout.rdbuf(devNull.rdbuf());

    const double u = 10.0;
    {
        FlightConditions fc;
        fc.setVInf(point{u, 0.0, 0.0, 0});
        fc.setAltitude(ALTITUDE);
        fc.comp();
        RHO = fc.getDensity();
    }
    const double q = 0.5*RHO*u*u;
    std::printf("Perfil: c_alpha = %.4f, alpha_0L = %.4f rad -> c0 = %.5f; rho(%.0f m) = %.4f\n",
                CALPHA, ALPHA0L, C0, ALTITUDE, RHO);

    const double alpha = toRad(5.0);
    const double ds = 0.01;                       // Euler estable para n = 21 si ds < ~0.049

    // ---- Caso 1: limite 2D
    Result r2 = runStep(1.0e4, 21, u, alpha, 30.0, ds, "../results/ullt_2d.h5");
    const double l2D = q*CHORD*(CALPHA*alpha + C0);
    std::printf("\n== Caso 1, limite 2D (b = 1e4): l_mid/(q c (c_alpha alpha + c0)) vs Wagner ==\n");
    std::printf("   s      ULLT      Wagner    dif\n");
    for(double sp : {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0}){
        const size_t k = static_cast<size_t>(std::lround(sp/ds));
        const double phi = 1.0 - 0.165*std::exp(-0.0455*sp) - 0.335*std::exp(-0.3*sp);
        const double val = r2.lMid[k]/l2D;
        std::printf("%5.1f   %.5f   %.5f   %+.1e\n", sp, val, phi, val - phi);
    }

    // ---- Caso 2: AR = 10, convergencia a Multhopp estacionario
    const double b = 10.0;
    {
        const size_t n = 21;
        Result r3 = runStep(b, n, u, alpha, 200.0, ds, "../results/ullt_ar10.h5");
        const Eigen::VectorXd lSteady = steadyLift(n, b, u, alpha);
        const std::vector<double> yv = yCoords(n, b);
        const Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(yv.data(), n);
        const double LSteady = trapz(y, lSteady);
        std::printf("\n== Caso 2, AR = 10, n = 21: estado estacionario ==\n");
        std::printf("               ULLT(s=200)   Multhopp     dif rel\n");
        std::printf("l_mid [N/m]    %10.4f   %10.4f   %+.1e\n",
                    r3.lMid.back(), lSteady(n/2), r3.lMid.back()/lSteady(n/2) - 1.0);
        std::printf("L     [N]      %10.4f   %10.4f   %+.1e\n",
                    r3.L.back(), LSteady, r3.L.back()/LSteady - 1.0);
    }

    // ---- Condiciones iniciales para los casos 3 y 4: estacionario con torsion nula (alpha = 0, solo comba)
    const size_t nDemo = 41;                      // |lambda|max ~ 122 -> Euler estable si ds < ~0.016
    const double dsDemo = 0.005;
    const Eigen::VectorXd lOwn = steadyLift(nDemo, b, u, 0.0);
    Eigen::VectorXd lInit = lOwn;
    try{
        lInit = phillipsSteadyLift(nDemo, b, u);
        std::printf("\n== Estacionario inicial (alpha = 0): PhillipsPhy vs Multhopp del ULLT ==\n");
        std::printf("l_mid  Phillips = %.4f N/m   ULLT = %.4f N/m   dif rel = %+.2e\n",
                    lInit(nDemo/2), lOwn(nDemo/2), lInit(nDemo/2)/lOwn(nDemo/2) - 1.0);
        std::printf("(la diferencia aparece como un transitorio inicial hacia el estacionario del ULLT)\n");
    } catch(const std::exception& e){
        std::printf("\nAVISO: PhillipsPhy fallo (%s); uso el estacionario del propio ULLT\n", e.what());
    }

    // ---- Caso 3: pulso gaussiano de cabeceo de 2 grados, ancho 2 semicuerdas, centrado en s = 10
    {
        const double A = toRad(2.0), s0 = 10.0, w = 2.0;
        PitchForcing f;
        f.alpha  = [=](double s){ const double x = (s - s0)/w; return A*std::exp(-x*x); };
        f.alphaS = [=](double s){ const double x = (s - s0)/w; return -2*A*x*std::exp(-x*x)/w; };
        Series r = runPitchDemo(b, nDemo, u, 2.5, 0.0, f, lInit, 80.0, dsDemo, 10, "../results/ullt_pitchPulse.h5");

        double lPk = -1e300, sPk = 0, nbPk = 0;
        for(size_t k = 0; k < r.s.size(); k++){
            const double dl = r.l[k] - lInit(r.i0);
            if(dl > lPk){ lPk = dl; sPk = r.s[k]; }
            nbPk = std::max(nbPk, std::abs(r.lNeighbour[k] - lInit(r.i0 + 1)));
        }
        std::printf("\n== Caso 3, pulso de cabeceo (2 grados, ancho %.0f semicuerdas, s0 = %.0f) en y = %.2f m ==\n",
                    w, s0, r.y0);
        std::printf("pico de Dl en la estacion   %8.3f N/m en s = %.2f (antes que el pico de alpha)\n", lPk, sPk);
        std::printf("pico |Dl| en la vecina      %8.3f N/m\n", nbPk);
        std::printf("Dl al final (s = 80)        %8.2e N/m\n", r.l.back() - lInit(r.i0));
    }

    // ---- Caso 4: cabeceo senoidal de 2 grados en la misma estacion, k = 0.3
    //      Arranque suave con envolvente E(s) = 1 - exp(-s/tau): alpha(0) = alpha'(0) = 0, asi no hay
    //      arranque impulsivo de la velocidad de cabeceo (que daria un pico de -dW_y/ds en el primer cuadro).
    {
        const double A = toRad(2.0), kRed = 0.3, tau = 5.0;
        PitchForcing f;
        f.alpha  = [=](double s){ return A*(1 - std::exp(-s/tau))*std::sin(kRed*s); };
        f.alphaS = [=](double s){
            const double e = std::exp(-s/tau);
            return A*(e/tau*std::sin(kRed*s) + (1 - e)*kRed*std::cos(kRed*s));
        };
        const double period = 2*M_PI/kRed;
        const double sMax = 8*period;
        Series r = runPitchDemo(b, nDemo, u, 2.5, 0.0, f, lInit, sMax, dsDemo, 20, "../results/ullt_pitchSine.h5");

        // Respuesta periodica sobre los 2 ultimos periodos
        double amp, ph, mean, ampA, phA, meanA;
        harmonic(r.s, r.l, kRed, sMax - 2*period, sMax, amp, ph, mean);
        harmonic(r.s, r.alpha, kRed, sMax - 2*period, sMax, ampA, phA, meanA);
        const double lQS = q*CHORD*CALPHA*A;      // 2D cuasiestacionario, sin efectos 3D
        std::printf("\n== Caso 4, cabeceo senoidal (2 grados, k = %.2f) en y = %.2f m ==\n", kRed, r.y0);
        std::printf("amplitud de l en la estacion  %8.3f N/m  (= %.3f del 2D cuasiestacionario)\n", amp, amp/lQS);
        std::printf("fase de l respecto a alpha    %+8.1f grados (positivo = adelanto)\n", ph - phA);
        std::printf("l medio en la estacion        %8.3f N/m  (estacionario inicial %.3f)\n", mean, lInit(r.i0));
    }

    std::printf("\nResultados en ../results/ullt_2d.h5, ullt_ar10.h5, ullt_pitchPulse.h5 y ullt_pitchSine.h5\n");

    std::cout.rdbuf(coutOld);
    return 0;
}
