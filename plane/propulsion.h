#ifndef PROPULSION_H
#define PROPULSION_H

#include <QObject>
#include <chrono>
#include "plane/planemodel.h"
#include "plane/earthandatmosphere.h"

#include <gsl/gsl_math.h>
#include <gsl/gsl_spline.h>
#include "config/config.h"
#include "data/datamanager.h"

using namespace std;

namespace planesimulator {

class PlaneModel;
class EarthAndAtmosphere;

class Propulsion : public QObject
{
    Q_OBJECT
public:
    explicit Propulsion(QObject *parent = nullptr);
    Propulsion(QObject *parent = nullptr, PlaneModel *p = nullptr, EarthAndAtmosphere *e = nullptr);
    ~Propulsion();

    void initialPropusion();

    void getProp(double *F, double *M);
    float fuelMass, fuelflow;

signals:

public slots:

private:
    PlaneModel *plane;
    EarthAndAtmosphere *eaa;
    DataManager *dm;
    chrono::steady_clock::time_point timeStepLast;
    double Omega, TSL, pSL, Jeng;
    double EPS;
    double MAPmin, Rprop, Jprop;
    double MAPTable[9];
    double RPMTable[9];
    double FuelFlowTable[9][9];
    double PowerTable[9][9];
    double JTable[16], CTTable[16], CPTable[16];
    double CT, CP;

    double MAP,RPM,power,torque,airflow;
    double Fprop, Mprop;

    gsl_spline2d *spline1, *spline;
    gsl_interp_accel *xacc, *xacc1, *acc2;
    gsl_interp_accel *yacc, *yacc1, *acc3;
    gsl_spline *spline_cubic, *spline_cubic1;

    double CalDotOmega();
    double CalDotOmega(double X);
    void CalcOmegaIntegration();
    void CalPiston(double P, double T, double Thr, double Mix);
    void CalPropeller(double airSpeed, double rho);


};
}
#endif // PROPULSION_H
