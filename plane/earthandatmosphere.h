#ifndef EARTHANDATMOSPHERE_H
#define EARTHANDATMOSPHERE_H

#include <QObject>
#include "data/logSystem.h"
#include <gsl/gsl_math.h>
#include <gsl/gsl_spline2d.h>
#include <gsl/gsl_spline.h>
#include "GeomagnetismHeader.h"
//#include "EGM9615.h"
#include "chrono"
#include "planemodel.h"

namespace planesimulator {

class PlaneModel;
class Frame;

class EarthAndAtmosphere : public QObject
{
    Q_OBJECT
public:
    explicit EarthAndAtmosphere(QObject *parent = nullptr);
    EarthAndAtmosphere(QObject *parent = nullptr, PlaneModel *p = nullptr, Frame *f = nullptr);
    ~EarthAndAtmosphere();

    void initialEAA();
    void GetWGS84(double *Rm, double *Rn, double *G);
    void GetWMM(float *mag);//magnetic field vector,[3]
    void GetECEF(double *ECEF);
    void GetAtmosphere();
    float p, t, rho, soundSpeed;

signals:

public slots:
private:

    PlaneModel *plane;
    Frame *frame;
    logSystem *log;

    double egm96Tabel[360][181];
    double egm96LatTabel[181];
    double egm96LonTabel[360];
    double tempTabel[861];
    double pressureTabel[861];
    double densityTabel[861];
    double vsTabel[861];
    double atmosphereAltTabel[861];

    gsl_spline2d *splineEgm96;
    gsl_interp_accel *xaccEgm96, *yaccEgm96;
    gsl_spline *spline_cubic1, *spline_cubic2, *spline_cubic3, *spline_cubic4;
    gsl_interp_accel *acc1, *acc2, *acc3, *acc4;

    void GetEgm96Alt(double *MSLAlt);

};

}
#endif // EARTHANDATMOSPHERE_H
