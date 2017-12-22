#ifndef PLANEMODEL_H
#define PLANEMODEL_H

#include <QObject>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <QByteArray>

#include "data/datamanager.h"
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_interp2d.h>
//#include <gsl/gsl_spline2d.h>
#include "plane/frame.h"
#include "plane/propulsion.h"
#include "plane/earthandatmosphere.h"
#include "config/config.h"

using namespace std;

namespace planesimulator {

class Frame;
class Propulsion;
class EarthAndAtmosphere;

class PlaneModel : public QObject
{
    Q_OBJECT
public:
    explicit PlaneModel(QObject *parent = 0);

    void InitialPlane();
    void SetCtrller(double *ctrller);

    //states of plane
    double phi, theta, psi;
    double p, q, r;
    double alpha, beta;
    double dotalpha, dotbeta;
    double da,de,dr,df,thr;
    double V;
    double u, v, w;//v in body
    double vn, ve, vd;//v in reference;
    double h, posn, pose;//position in meter
    double latitude, longtitude;
    double G;
    double pdyn;
    Frame *frame;
    Propulsion *prop;
    EarthAndAtmosphere *eaa;

    float simuTime;
    bool isRun;

signals:

public slots:
    void RunModel();//主计时器调用

private:

    DataManager *dm;
    int statesNumber;
    chrono::steady_clock::time_point timeStepLast;
    chrono::steady_clock::time_point timefuelFlowLast;

    double b, S, MAC;

    //Parameters
    double CL,CD,CY,Cl,Cm,Cn;
    double CL0, CLa, CLdf, CLde, CLalphadot, CLq;
    double CLmind, CD0, CDdf, CDde, CDda, CDdr, osw;
    double CYbeta, CYda, CYdr;
    double Clbeta, Clda, Cldr, Clp, Clr;
    double Cm0, Cma, Cmdf, Cmde, Cmalphadot, Cmq;
    double Cnbeta, Cnda, Cndr, Cnp, Cnr;

    //forces and moments
    double Fx,Fy,Fz,Mx,My,Mz;
    double Faerox, Faeroy, Faeroz, Maerox, Maeroy, Maeroz;
    double Fprop, Mprop;
    double mempty, mgross, CGempty[3], CGgross[3], Jempty[4], Jgross[4];
    double Mass, CGpos[3], J[4];
    double rHub[3], rAC[3];

    //earth and air
    //double pdyn;
    double Rm, Rn;
    double windx,windy,windz;//wind velocity in body

    //input
    double deTable[8];
    double alphaTabel[10];
    double CDTabel[10][8];

    void Solver();
    void DifferentialEquation(double *dotX, double *X);
    void GetParameters();
    void GetAeroForcesAndMoments();
    void GetInertia();
    void GetTotalForceAndTotalMoments();
    void GetAlphaAndBeta();

    QStringList *tmp;
};

}
#endif // PLANEMODEL_H
